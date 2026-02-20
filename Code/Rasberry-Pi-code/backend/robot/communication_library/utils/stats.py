"""
stats.py – Liczniki wyników testów (pass/fail per kategoria)
=============================================================
TestStats agreguje wyniki testów i generuje czytelne podsumowanie
na koniec sesji testowej.
"""


class TestStats:
    """
    Kolekcja statystyk testów z podziałem na kategorie.

    Przykład:
        stats = TestStats()
        stats.add("01-Write-Valid", passed=True)
        stats.add("03-Bad-StartByte", passed=False)
        stats.print_summary()
    """

    def __init__(self):
        self.total   = 0
        self.passed  = 0
        self.failed  = 0
        self._cats:  dict[str, dict[str, int]] = {}

    def add(self, category: str, passed: bool) -> None:
        """
        Rejestruje wynik pojedynczego testu.

        Parametry
        ----------
        category : str
            Nazwa kategorii (np. "01-Write-Valid").
        passed : bool
            True = test zaliczony, False = test niezaliczony.
        """
        self.total += 1
        if passed:
            self.passed += 1
        else:
            self.failed += 1

        if category not in self._cats:
            self._cats[category] = {"passed": 0, "failed": 0}
        self._cats[category]["passed" if passed else "failed"] += 1

    def reset(self) -> None:
        """Zeruje wszystkie liczniki."""
        self.total  = 0
        self.passed = 0
        self.failed = 0
        self._cats.clear()

    def print_summary(self, printer=print) -> None:
        """
        Drukuje tabelaryczne podsumowanie wyników.

        Parametry
        ----------
        printer : callable
            Funkcja drukująca (domyślnie print).
        """
        printer("\n" + "=" * 70)
        printer("  PODSUMOWANIE TESTÓW")
        printer("=" * 70)
        printer(f"  Łącznie testów : {self.total}")
        printer(f"  Udanych        : {self.passed}")
        printer(f"  Nieudanych     : {self.failed}")
        if self.total > 0:
            printer(f"  Wskaźnik sukc. : {self.passed / self.total * 100:.1f}%")
        printer("\n" + "-" * 70)
        printer(f"  {'Kategoria':<38} {'Pass':>5} {'Total':>6} {'%':>7}")
        printer("-" * 70)

        for cat in sorted(self._cats):
            r     = self._cats[cat]
            total = r["passed"] + r["failed"]
            pct   = r["passed"] / total * 100 if total else 0.0
            status = "OK " if r["failed"] == 0 else "!!!"
            printer(f"  [{status}] {cat:<35} {r['passed']:>5} {total:>6} {pct:>6.1f}%")

        printer("=" * 70)
        if self.failed == 0:
            printer("  WSZYSTKIE TESTY PRZESZŁY POMYŚLNIE!")
        else:
            printer(f"  UWAGA: {self.failed} testów wymaga uwagi")
        printer("=" * 70 + "\n")

    @property
    def categories(self) -> dict:
        """Zwraca słownik statystyk per kategoria (tylko do odczytu)."""
        return dict(self._cats)
