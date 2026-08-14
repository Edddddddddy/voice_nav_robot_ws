# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Internal scan/TF coverage helpers for Mapping acceptance."""


_ROS_SECOND_NS = 1_000_000_000
_MINIMUM_RETAINED_ROUTE_SCANS = 40
_REQUIRED_SCAN_TRANSFORMS = (
    ('map', 'laser_link'),
    ('base_footprint', 'laser_link'),
)


def _route_scan_candidates(samples, *, route_started_ns, route_completed_ns):
    if route_completed_ns <= route_started_ns:
        raise AssertionError('route ROS interval must be positive')

    first_bucket = route_started_ns // _ROS_SECOND_NS
    last_bucket = (route_completed_ns - 1) // _ROS_SECOND_NS
    candidates_by_bucket = {}
    for stamp_ns, scan in sorted(samples, key=lambda sample: sample[0]):
        if not route_started_ns <= stamp_ns < route_completed_ns:
            continue
        bucket = stamp_ns // _ROS_SECOND_NS
        candidates_by_bucket.setdefault(bucket, []).append((stamp_ns, scan))

    missing_buckets = [
        bucket
        for bucket in range(first_bucket, last_bucket + 1)
        if bucket not in candidates_by_bucket
    ]
    if missing_buckets:
        rendered = ', '.join(str(bucket) for bucket in missing_buckets)
        raise AssertionError(f'missing ROS scan buckets: {rendered}')

    return tuple(
        (bucket, tuple(candidates_by_bucket[bucket]))
        for bucket in range(first_bucket, last_bucket + 1)
    )


def verify_route_scan_transform_coverage(
    samples,
    *,
    route_started_ns,
    route_completed_ns,
    lookup_transform,
):
    """Require scan/TF coverage for every ROS-second route bucket."""
    candidate_buckets = _route_scan_candidates(
        samples,
        route_started_ns=route_started_ns,
        route_completed_ns=route_completed_ns,
    )
    if len(candidate_buckets) < _MINIMUM_RETAINED_ROUTE_SCANS:
        raise AssertionError(
            'route scan sample count below minimum: '
            f'actual={len(candidate_buckets)}, '
            f'required={_MINIMUM_RETAINED_ROUTE_SCANS}'
        )
    retained = []
    failures = []
    for bucket, candidates in candidate_buckets:
        candidate_failures = []
        for stamp_ns, scan in candidates:
            scan_failures = []
            for target, source in _REQUIRED_SCAN_TRANSFORMS:
                try:
                    lookup_transform(target, source, scan)
                except Exception as error:
                    scan_failures.append(
                        f'stamp_ns={stamp_ns}, {target}<-{source}: {error}'
                    )
            if not scan_failures:
                retained.append((bucket, stamp_ns, scan))
                break
            candidate_failures.extend(scan_failures)
        else:
            failures.append(
                f'bucket={bucket}, no scan has complete transform coverage: '
                + '; '.join(candidate_failures)
            )
    if failures:
        raise AssertionError(
            'route scan transform coverage failed: ' + '; '.join(failures)
        )
    return tuple(retained)
