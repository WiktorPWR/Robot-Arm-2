"""
log.py – Logowanie wymiany bajtów SPI (MOSI / MISO)
======================================================
CommunicationLog gromadzi wszystkie transakcje SPI z danej sesji
(jednej operacji WRITE lub READ) i umożliwia ich czytelne wyświetlenie.

Używany w protokole do debugowania – przekazywany opcjonalnie do każdej
funkcji niskiego poziomu (transport.xfer, do_handshake itp.).
"""


class CommunicationLog:
    """
    Kontener transakcji SPI z możliwością wydruku przepływu komunikacji.

    Przykład użycia:
        log = CommunicationLog()
        transport.xfer([0x97], log, "SYN")
        log.print_flow()
    """

    def __init__(self):
        self._entries: list[dict] = []

    def add(self, step: str, mosi: list[int], miso: list[int]) -> None:
        """
        Dodaje wpis do logu.

        Parametry
        ----------
        step : str
            Opis kroku (np. "1. SYN → slave").
        mosi : list[int]
            Bajty wysłane przez mastera.
        miso : list[int]
            Bajty odebrane od slave'a.
        """
        self._entries.append({
            "step": step,
            "mosi": list(mosi),
            "miso": list(miso),
        })

    def clear(self) -> None:
        """Czyści wszystkie zapisane wpisy."""
        self._entries.clear()

    def print_flow(self, printer=print) -> None:
        """
        Drukuje czytelny dump całej wymiany bajtów.

        Parametry
        ----------
        printer : callable
            Funkcja drukująca (domyślnie print).
            Można podać własną, np. rprint z modułu testowego.
        """
        printer("\n" + "╔" + "═" * 68 + "╗")
        printer("║" + " " * 22 + "FLOW KOMUNIKACJI" + " " * 30 + "║")
        printer("╚" + "═" * 68 + "╝")

        for i, entry in enumerate(self._entries, 1):
            printer(f"\n{'─' * 70}")
            printer(f"  Krok {i}: {entry['step']}")
            printer(f"{'─' * 70}")
            printer(f"  MOSI (Master→Slave): {[f'0x{b:02X}' for b in entry['mosi']]}")
            printer(f"  MISO (Slave→Master): {[f'0x{b:02X}' for b in entry['miso']]}")

        printer("\n" + "=" * 70 + "\n")

    def to_list(self) -> list[dict]:
        """Zwraca kopię listy wpisów (do własnego przetwarzania)."""
        return [dict(e) for e in self._entries]

    def __len__(self) -> int:
        return len(self._entries)

    def __repr__(self) -> str:
        return f"CommunicationLog({len(self._entries)} entries)"
