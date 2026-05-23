"""
Pure metrics computation for recorded topic streams.

Input: list of receive timestamps (nanoseconds, int).
Output: dict of metrics.

No ROS dependency, no I/O — easy to unit-test in isolation.
"""

from typing import List, Dict, Optional
import statistics


def compute_topic_metrics(
    timestamps_ns: List[int],
    expected_rate_hz: Optional[float] = None,
    gap_threshold_factor: float = 3.0,
) -> Dict:
    """
    Compute per-topic timing metrics.

    Parameters
    ----------
    timestamps_ns : list of int
        Receive timestamps in nanoseconds, monotonically non-decreasing.
    expected_rate_hz : float or None
        Expected publication rate. If provided, metrics include deviation.
    gap_threshold_factor : float
        Inter-arrival intervals above (median * factor) are flagged as gaps.

    Returns
    -------
    dict
        Computed metrics. See keys below.
    """
    n = len(timestamps_ns)
    if n == 0:
        return {
            'message_count': 0,
            'note': 'no messages',
        }

    if n == 1:
        return {
            'message_count': 1,
            'note': 'only one message — no interval metrics',
            'first_timestamp_ns': timestamps_ns[0],
            'last_timestamp_ns': timestamps_ns[0],
        }

    # Inter-arrival intervals in seconds
    intervals_s = [
        (timestamps_ns[i] - timestamps_ns[i - 1]) / 1e9
        for i in range(1, n)
    ]

    duration_s = (timestamps_ns[-1] - timestamps_ns[0]) / 1e9
    actual_rate_hz = (n - 1) / duration_s if duration_s > 0 else 0.0

    iat_mean = statistics.mean(intervals_s)
    iat_std = statistics.stdev(intervals_s) if len(intervals_s) > 1 else 0.0
    iat_median = statistics.median(intervals_s)
    iat_min = min(intervals_s)
    iat_max = max(intervals_s)

    # Percentiles (statistics.quantiles needs n>=2 data points)
    sorted_iat = sorted(intervals_s)
    p95 = _percentile(sorted_iat, 95)
    p99 = _percentile(sorted_iat, 99)

    # Jitter: std / mean (coefficient of variation, dimensionless)
    iat_jitter_cv = iat_std / iat_mean if iat_mean > 0 else 0.0

    # Gap detection: intervals significantly above median
    gap_threshold = iat_median * gap_threshold_factor
    gaps = [
        {'index': i, 'interval_s': iv}
        for i, iv in enumerate(intervals_s)
        if iv > gap_threshold
    ]

    result = {
        'message_count': n,
        'duration_s': round(duration_s, 6),
        'actual_rate_hz': round(actual_rate_hz, 3),
        'inter_arrival_time_s': {
            'mean': round(iat_mean, 6),
            'std': round(iat_std, 6),
            'median': round(iat_median, 6),
            'min': round(iat_min, 6),
            'max': round(iat_max, 6),
            'p95': round(p95, 6),
            'p99': round(p99, 6),
        },
        'jitter_coefficient_of_variation': round(iat_jitter_cv, 4),
        'gap_count': len(gaps),
        'gap_threshold_s': round(gap_threshold, 6),
        'gaps': gaps[:10],  # cap at 10 to keep JSON small
    }

    if expected_rate_hz is not None:
        deviation = (actual_rate_hz - expected_rate_hz) / expected_rate_hz
        result['expected_rate_hz'] = expected_rate_hz
        result['rate_deviation'] = round(deviation, 4)

    return result


def _percentile(sorted_data: List[float], pct: float) -> float:
    """Compute percentile via linear interpolation. Input must be sorted."""
    if not sorted_data:
        return 0.0
    if len(sorted_data) == 1:
        return sorted_data[0]
    k = (len(sorted_data) - 1) * (pct / 100.0)
    f = int(k)
    c = min(f + 1, len(sorted_data) - 1)
    if f == c:
        return sorted_data[f]
    return sorted_data[f] + (sorted_data[c] - sorted_data[f]) * (k - f)
