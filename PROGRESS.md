# PROGRESS — płaska lista kroków (bez tygodni)

> To jest nasza pamięć projektu między sesjami. Zawsze zaczynamy sesję od
> sprawdzenia tego pliku: co jest zrobione, co jest "AKTUALNY KROK".
> Pełny opis zadania: `thesis/plan_original.md`. Założenia modelu:
> `docs/assumptions.md`.
>
> Mechanika: kroki konfiguracyjne/dokumentacyjne robi Claude bezpośrednio.
> Kroki z kodem fizyki/regulatorów/MPC trafiają do `CODE_TO_BE_REWRITTEN.py`
> (plik-scratchpad w katalogu głównym, w `.gitignore`) — tam jest kod z komentarzami
> po angielsku do ręcznego przepisania do właściwego pliku projektu. Po przepisaniu
> i odpaleniu, zgłaszasz wynik, a ja odhaczam krok i podaję następny.

**AKTUALNY KROK: 34**

## Faza A — Setup i porządki

- [x] 1. Uporządkowanie starych plików: `main.py` usunięty (boilerplate PyCharm),
      stary tutorial 1D zarchiwizowany do `thesis/archive_pycharm_1d_tutorial.md`,
      `README.md` zastąpiony krótkim przeglądem projektu.
- [x] 2. Oryginalny opis pracy zapisany do `thesis/plan_original.md`.
- [x] 3. `pyproject.toml` wypełniony (metadane + zależności: numpy, scipy,
      matplotlib, casadi, pytest jako dev-extra).
- [x] 4. Smoke test instalacji: `import numpy, matplotlib, pytest` działa
      (numpy 2.5.4, matplotlib 3.11.2, pytest 9.1.1, Python 3.14.8).
- [x] 5. CasADi zainstalowany (3.8.1) i przetestowany: toy problem optymalizacji
      (`min (x-1)^2+(y-2)^2`) rozwiązany przez IPOPT, wynik `x=1, y=2` ✓.
      Brak problemów z instalacją na tej maszynie (macOS, Python 3.14, arm64).
- [x] 6. Szkielet pakietu: `src/rocket_landing/__init__.py` utworzony.
- [x] 7. `PROGRESS.md` i `CODE_TO_BE_REWRITTEN.py` utworzone, `.gitignore`
      zaktualizowany (ignoruje scratchpad + standardowe śmieci Python).
- [x] 8. Założenia modelu zapisane w `docs/assumptions.md`.

*Punkt kontrolny Fazy A: środowisko gotowe, żadnych niespodzianek z instalacją
CasADi/IPOPT, struktura katalogów na miejscu.* ✅ ZALICZONY

## Faza B — Fizyka i symulator (bez regulatora)

- [x] 9. Klasa parametrów rakiety (`src/rocket_landing/params.py`): masa, masa
      własna, max ciąg, Isp, g0, max kąt odchylenia — zwykłe pola Python.
      Sprawdzone w konsoli Python: `print(p.initial_mass, p.dry_mass,
      p.max_thrust, p.max_tilt_angle)` → `25000.0 22000.0 400000.0
      0.3490658503988659` ✓.
- [x] 10. Funkcja grawitacji / swobodny spadek (najprostsza możliwa `dynamics`).
      Zweryfikowana w konsoli, poprawiony błąd transkrypcji (acceleration miał
      2 elementy zamiast 3, przez literówkę `0.0 - params.g` zamiast
      `0.0, 0.0 - params.g`).
- [x] 11. Funkcja wektora kierunku ciągu `d(alpha, beta)` + ręczne sprawdzenie
      długości = 1. Zweryfikowana w konsoli: `d(0,0)=[0,0,1]`,
      `d(max_tilt,0)=[0.342, 0, 0.9397]`, `norm(d(0.4,-0.25))=1.0` ✓.
- [x] 12. Pełna funkcja `dynamics()`: grawitacja + ciąg + zużycie masy (bez
      oporu powietrza). Zweryfikowana w konsoli: zero-thrust == free-fall,
      hover test: przyspieszenie ~0, mass_rate = -83.33 kg/s ✓.
- [x] 13. Integrator Euler (najprostszy, do szybkiej weryfikacji).
      Zweryfikowany w konsoli: swobodny spadek, 10 kroków po dt=0.1s.
- [x] 14. Ręczny skrypt: swobodny spadek, wypisywanie wysokości — porównanie z
      ręcznym rachunkiem. vz zgodne idealnie (-9.80665), z zawyżone o ~0.49 m
      (znany, oczekiwany błąd metody Eulera) ✓.
- [x] 15. Ręczny skrypt: lot pionowy ze stałym ciągiem — porównanie z ręcznym
      rachunkiem. Euler zgodny z ręcznym rachunkiem (z, vz, m) do zaokrąglenia
      float; m idealnie zgodne (brak błędu Eulera dla masy, jak oczekiwano) ✓.
- [x] 16. Pierwsze testy pytest (`test_dynamics.py`): przyspieszenie w swobodnym
      spadku, zachowanie masy przy zerowym ciągu. 2 passed ✓. Poprawiono
      drugi test: `== 0.0` zamiast `pytest.approx(0.0)` (approx maskowałby
      prawdziwy błąd, bo 0.0/x jest zawsze dokładnie zerem).
- [x] 17. Integrator RK4 (`integrators.py`), osobno od Eulera. Zweryfikowany
      w konsoli: swobodny spadek 1s, z i vz zgodne z dokładną formułą
      PRAKTYCZNIE idealnie (w przeciwieństwie do błędu Eulera ~0.49 m).
      Poprawiono błąd transkrypcji (`k1 + 2 + k2` zamiast `k1 + 2*k2`).
- [x] 18. Test porównujący RK4 vs Euler na swobodnym spadku (dokładność).
      1 passed - błąd RK4 ponad 1000x mniejszy niż błąd Eulera ✓.
- [x] 19. Detekcja kontaktu z ziemią (`z <= 0` → stop symulacji).
      Zweryfikowana w konsoli: swobodny spadek z 50 m, dt=0.01.
      321 zarejestrowanych stanów, final time ≈ 3.20 s (zgodne z dokładnym
      czasem spadku 3.1933 s + jeden krok "przeskoku" przez ziemię),
      final z ≈ -0.21 m (mały ujemny przeskok, oczekiwany przy tym dt) ✓.
- [x] 20. Ograniczenia ciągu i kąta odchylenia (`actuators.py`).
      Zweryfikowana w konsoli: wild control (999999, 5, -5) przycięty do
      (400000, 0.349.., -0.349..), legalny control (200000, 0.1, -0.1)
      wraca BEZ ZMIAN ✓.
- [x] 21. Pierwsze wykresy: wysokość/prędkość/masa w czasie dla ustalonego
      profilu ciągu. Zweryfikowane wizualnie (`scripts/plot_profile.py`):
      wysokość gładko spada 500→0 w ~25 s, vz coraz bardziej ujemne
      (nigdy nie zwalnia - zbyt słaba tego ciągu, jak przewidziano),
      masa idealnie liniowa 25000→~23300 kg ✓.
- [x] 22. Pierwszy wykres trajektorii 3D (matplotlib) dla ręcznie zadanego
      profilu ciągu z komponentem bocznym. Zweryfikowany wizualnie
      (`plot_3d_trajectory()`): trajektoria od (0,0,800) do lądowania
      z wyraźnym znoszeniem w x i y, zgodnie z przewidywaniem (kierunek
      ciągu stały => trajektoria prawie idealnie prosta w przestrzeni,
      tylko siła się zmienia w czasie, nie kierunek) ✓.

*Punkt kontrolny Fazy B: symulator fizyczny działa i jest przetestowany, bez
żadnego regulatora.* ✅ ZALICZONY

## Faza C — Regulator bazowy PID

- [x] 23. Definicja celu (lądowisko, zerowa prędkość) w `reference.py`.
      Zweryfikowana w konsoli: `target.position = [0,0,0]`,
      `target.velocity = [0,0,0]` ✓.
- [x] 24. Prosty PID na prędkość pionową (`controllers/pid.py`). Zweryfikowany
      w konsoli: 3x ta sama error=5.0 → 10.25, 10.5, 10.75 (zgodne z ręcznym
      rachunkiem, narastanie o 0.25 dzięki członowi I) ✓.
- [x] 25. Test PID w zamkniętej pętli — tylko ruch pionowy, ręczne dostrojenie.
      Po drodze znaleziony i naprawiony realny bug: brak zatrzymania
      symulacji przy wyczerpaniu paliwa (`mass <= dry_mass`) powodował,
      że masa schodziła w głęboki minus, a dzielenie przez ujemną masę
      w `dynamics()` dawało fizycznie absurdalne wyniki (vz=-7053 m/s).
      Po poprawce i kilku iteracjach (kp/ki/kd): finalnie kp=1500,
      ki=50, kd=800, start z vz=-50 m/s na 500 m → lądowanie po 18.6 s,
      touchdown vz=-8.06 m/s, mass=23164 kg (paliwo z zapasem). Nie jest
      to jeszcze "miękkie" lądowanie (cel z planu: ≤1 m/s), ale pętla
      zamknięta działa stabilnie, bez rozbiegania ✓. Dalsze dostrojenie
      w kolejnych krokach.
- [x] 26. Rozszerzenie PID na pozycję poziomą (x, y) — `pid_x`/`pid_y` na błąd
      pozycji (kp=500, kd=2000 każdy), ciąg wciąż z `pid_vz`, `alpha`/`beta`
      z małokątowego przybliżenia `desired_force / thrust`.
- [x] 27. Pętla symulacji w zamkniętym obiegu (`simulate_pid` w `simulation.py`):
      stan → trzy PID → fizyka. Po drodze znaleziony i naprawiony realny
      bug podczas testowania (nie literówka transkrypcji, ale logiczny błąd):
      `desired_fy` liczone przez `pid_x.compute(...)` zamiast `pid_y.compute(...)`
      — przy starcie z `y=0` błąd był niewidoczny (błąd y = 0 od początku),
      ujawnił się dopiero po teście ze startem `x=100, y=50`. Po poprawce:
      start (x=100, y=50, z=500, vz=-50) → lądowanie po 17.85 s,
      touchdown x=-27.3 m, y=-13.7 m (obie osie z wyraźnym przeregulowaniem/
      overshoot, symetrycznie — te same wzmocnienia), vz=-9.59 m/s (wciąż
      nie "miękkie"), masa=23228 kg (paliwo z zapasem). Pętla zamknięta
      działa stabilnie na obu osiach, bez rozbiegania ✓. Dalsze dostrojenie
      (przeregulowanie x/y, zbyt duże vz przy dotknięciu) odłożone na
      kolejne kroki/fazy.
- [x] 28. Pierwsza wersja `metrics.py`: błąd lądowania, prędkość przyziemienia,
      zużycie paliwa. Trzy funkcje (`landing_position_error`,
      `landing_speed`, `fuel_used`) + wrapper `landing_summary`. Po drodze
      drobna poprawka transkrypcji: `fuel_used` zwracało `np.float64` bez
      `float(...)` (niezgodne z pozostałymi funkcjami). Zweryfikowane na
      scenariuszu z kroku 27 (x=100, y=50, z=500, vz=-50): position_error_m
      ≈30.58 m, touchdown_speed_mps ≈10.83 m/s (więcej niż |vz|=9.59,
      poprawnie liczy też vx/vy), fuel_used_kg ≈1772.47 kg, flight_time_s
      ≈17.85 s — wszystko zgodne z ręcznym oszacowaniem ✓.
- [x] 29. Wykres i ocena pierwszego pełnego lądowania z PID (3D + przebiegi
      czasowe) — `plot_full_pid_landing()` w `scripts/plot_profile.py`,
      ten sam scenariusz co kroki 27/28 (x=100, y=50, z=500, vz=-50).
      Zweryfikowane wizualnie: wysokość gładko 500→0, x i y schodzą od
      100/50, przecinają zero ok. t≈13.5 s i kontynuują w minus
      (przeregulowanie widoczne na wykresie), vz rośnie z -50 do ok. -9.5;
      na wykresie 3D trajektoria mija czarny krzyżyk (cel) i ląduje w
      czerwonym punkcie za nim. Wydrukowany `landing_summary` idealnie
      zgodny z krokiem 28 (flight_time_s=17.85, position_error_m=30.58,
      touchdown_speed_mps=10.83, fuel_used_kg=1772.47) — potwierdza, że
      wykres pokazuje ten sam, już zweryfikowany lot ✓.

*Punkt kontrolny Fazy C: PID samodzielnie prowadzi rakietę do (niedoskonałego)
lądowania od początku do końca.* ✅ ZALICZONY (przeregulowanie x/y i zbyt duże
vz przy dotknięciu to znane ograniczenia tego baseline'u, do porównania z MPC
w Fazie D/E)

## Faza D — Przejście na MPC (CasADi)

- [x] 30. Symboliczna wersja dynamiki w CasADi (osobna implementacja, nie import
      z `dynamics.py`) — `casadi_dynamics()` w nowym `controllers/mpc.py`,
      przepisana niezależnie (`ca.sin`/`ca.cos`/`ca.vertcat` zamiast
      `np.sin`/`np.cos`/`np.concatenate`), żeby model MPC i model symulatora
      nie współdzieliły tego samego kodu (ważne pod kątem eksperymentu 45 —
      niezgodność modelu). Zweryfikowane w konsoli przez `ca.Function`:
      hover (thrust=mass*g, alpha=beta=0) → przyspieszenie [0,0,0],
      mass_rate=-83.33 kg/s, zgodne z `dynamics()` do pełnej precyzji;
      dwa dodatkowe przypadki z nachyloną tą samą formułą (różne
      thrust/alpha/beta) — wyniki `casadi_dynamics` i `dynamics` identyczne
      w każdym z nich ✓.
- [x] 31. Dyskretyzacja RK4 w CasADi (`casadi_rk4_step()` w `controllers/mpc.py`,
      osobna implementacja integratora od `rk4_step()` w `integrators.py`,
      wywołuje `casadi_dynamics` niezależnie od `rk4_step`). Zweryfikowane
      przez `ca.Function`: hover (dt=0.1) i naklon thrust (dt=0.5) —
      wyniki `casadi_rk4_step` i `rk4_step` identyczne do pełnej precyzji
      floatów w obu przypadkach ✓.
- [x] 32. Funkcja kosztu jako wyrażenia CasADi (błąd pozycji, prędkości,
      sterowania, Δu): `stage_cost()` i `terminal_cost()` w `controllers/mpc.py`.
      Zweryfikowane ręcznym przeliczeniem (`scripts/steps/check_step32.py`):
      `stage_cost` ≈ 50230.25, `terminal_cost` = 61.8 — zgodne do pełnej
      precyzji floatów ✓.
- [x] 33. Budowa NLP: zmienne decyzyjne (stan+sterowanie na horyzoncie),
      ograniczenia dynamiki: `build_mpc_nlp()` w `controllers/mpc.py` (CasADi
      `Opti`, `X` 7×(N+1), `U` 3×N, ograniczenia `X[:,k+1] == casadi_rk4_step(...)`).
      Zweryfikowane (`scripts/steps/check_step33.py`): zadanie z w pełni
      przypiętymi zmiennymi (start + stałe sterowanie "hover") rozwiązane przez
      IPOPT, trajektoria identyczna z ręcznym łańcuchem `casadi_rk4_step` —
      max różnica 0.0 ✓.
- [ ] 34. Ograniczenia: ciąg min/max, kąt odchylenia, wysokość ≥ 0, masa ≥ masa
      własna.
      **← następny krok w `CODE_TO_BE_REWRITTEN.py`**
- [ ] 35. Rozwiązanie jednego NLP w pętli otwartej (bez MPC jeszcze) — podgląd
      trajektorii.
- [ ] 36. Pętla MPC z przesuwanym horyzontem: w każdym kroku rozwiąż NLP od
      aktualnego stanu, zastosuj pierwsze sterowanie, zasymuluj krok, powtórz.
- [ ] 37. Ciepły start (warm-start) z poprzedniego rozwiązania.
- [ ] 38. Obsługa niepowodzenia solvera (logowanie, fallback).
- [ ] 39. Pierwsze pełne nominalne lądowanie z MPC + wykresy.

*Punkt kontrolny Fazy D: MPC samodzielnie prowadzi rakietę do lądowania w
scenariuszu nominalnym.*

## Faza E — Eksperymenty i odporność

- [ ] 40. Standaryzacja scenariuszy startowych i zbierania metryk dla PID i MPC.
- [ ] 41. Migracja parametrów do `configs/*.yaml` (dopiero teraz — łatwe
      przełączanie scenariuszy między eksperymentami).
- [ ] 42. Eksperyment A: nominalne porównanie PID vs MPC, tabela wyników.
- [ ] 43. Dodanie wiatru (`environment.py`) i powtórzenie eksperymentu.
- [ ] 44. Błąd początkowej prędkości i masy — kolejne warianty eksperymentu C.
- [ ] 45. Niezgodność modelu MPC vs "prawdziwego" symulatora (inny ciąg/Isp).
- [ ] 46. Eksperyment B: siatka warunków początkowych, mapa sukces/porażka.
- [ ] 47. Eksperyment D: sweep parametrów MPC (horyzont, wagi, częstotliwość
      sterowania).
- [ ] 48. Eksperyment E: Monte Carlo (ziarno RNG zapisane), statystyki i wykresy.
- [ ] 49. (Jeśli czas pozwoli) opór aerodynamiczny.
- [ ] 50. (Jeśli czas pozwoli) rozszerzenie o orientację/kąt nachylenia korpusu.

*Punkt kontrolny Fazy E: pełna, powtarzalna matryca eksperymentów w
`experiments/`.*

## Faza F — Pisanie pracy i finisz

- [ ] 51. Weryfikacja numeryczna (czułość na krok czasowy, audyt naruszeń
      ograniczeń).
- [ ] 52. Rozdział: metodologia i model matematyczny.
- [ ] 53. Rozdział: sformułowanie i implementacja MPC.
- [ ] 54. Rozdział: eksperymenty i wyniki (tabele/wykresy z Fazy E).
- [ ] 55. Rozdział: wnioski i ograniczenia + finalny `README.md` z instrukcją
      odtworzenia.
- [ ] 56. Prezentacja, nagranie demo, krótki raport techniczny po angielsku
      (portfolio).
- [ ] 57. Próba obrony: pytania o niezbieżność MPC, niewystarczający ciąg,
      weryfikację modelu, kompromis dokładność/czas obliczeń.
