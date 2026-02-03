# protocol.py

class SlaveMasterProtocol:
    START_BYTE = 0xAA
    END_BYTE = 0xBB

    CMD_MOVE = 0x01
    CMD_HOMING = 0x02
    CMD_ESTOP = 0x03
    CMD_DIAGNOSTICS = 0x04
    CMD_HOMMING_STATUS = 0x05
    CMD_DIAGNOSTICS_STATUS = 0x06

    @staticmethod
    def calculate_crc(data_bytes):
        """Prosta suma kontrolna XOR."""
        crc = 0
        for byte in data_bytes:
            crc ^= byte
        return crc

    @staticmethod
    def create_frame(slave_id, command, data):
        """
        Tworzy ramkę: [START, ADDR, CMD, LEN, DATA..., CRC, END]
        """
        if isinstance(data, int):
            data = [data]
        
        length = len(data)
        # Budujemy środek ramki do obliczenia CRC
        payload = [slave_id, command, length] + list(data)
        crc = SlaveMasterProtocol.calculate_crc(payload)
        
        # Pełna ramka
        frame = [SlaveMasterProtocol.START_BYTE] + payload + [crc, SlaveMasterProtocol.END_BYTE]
        return frame