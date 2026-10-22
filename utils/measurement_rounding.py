"""Integer presentation of measurements; fitting/calibration retain full precision."""
from decimal import Decimal, ROUND_HALF_UP


def round_measurement(value) -> int:
    number = Decimal(str(value))
    if not number.is_finite():
        raise ValueError('Measurement must be finite')
    # Least-squares arithmetic can leave an exact half just below the tie.
    # Remove sub-nanounit noise before rounding the published measurement.
    number = number.quantize(Decimal('0.000000001'), rounding=ROUND_HALF_UP)
    return int(number.quantize(Decimal('1'), rounding=ROUND_HALF_UP))
