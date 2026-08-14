# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Geometry helpers for the frozen Mapping acceptance oracle."""

import math

_INTERSECTION_EPSILON = 1.0e-7


def boundary_points(shape, spacing):
    """Sample the complete planar collision boundary of one SDF shape."""
    if shape['kind'] == 'cylinder':
        count = max(16, math.ceil(2.0 * math.pi * shape['radius'] / spacing))
        return tuple(
            (
                shape['x'] + shape['radius'] * math.cos(2.0 * math.pi * index / count),
                shape['y'] + shape['radius'] * math.sin(2.0 * math.pi * index / count),
            )
            for index in range(count)
        )

    half_x = shape['size_x'] / 2.0
    half_y = shape['size_y'] / 2.0
    x_count = max(1, math.ceil(shape['size_x'] / spacing))
    y_count = max(1, math.ceil(shape['size_y'] / spacing))
    points = []
    for index in range(x_count + 1):
        x = shape['x'] - half_x + shape['size_x'] * index / x_count
        points.extend([(x, shape['y'] - half_y), (x, shape['y'] + half_y)])
    for index in range(y_count + 1):
        y = shape['y'] - half_y + shape['size_y'] * index / y_count
        points.extend([(shape['x'] - half_x, y), (shape['x'] + half_x, y)])
    return tuple(points)


def _sample_route(route, spacing):
    if len(route) == 1:
        return tuple(route)
    samples = []
    for start, finish in zip(route, route[1:]):
        distance = math.dist(start, finish)
        count = max(1, math.ceil(distance / spacing))
        for index in range(count):
            ratio = index / count
            samples.append(
                (
                    start[0] + (finish[0] - start[0]) * ratio,
                    start[1] + (finish[1] - start[1]) * ratio,
                )
            )
    samples.append(tuple(route[-1]))
    return tuple(samples)


def _box_entry_parameter(origin, target, shape):
    minimum_x = shape['x'] - shape['size_x'] / 2.0
    maximum_x = shape['x'] + shape['size_x'] / 2.0
    minimum_y = shape['y'] - shape['size_y'] / 2.0
    maximum_y = shape['y'] + shape['size_y'] / 2.0
    entry = 0.0
    exit_parameter = 1.0
    for start, delta, minimum, maximum in (
        (origin[0], target[0] - origin[0], minimum_x, maximum_x),
        (origin[1], target[1] - origin[1], minimum_y, maximum_y),
    ):
        if abs(delta) <= _INTERSECTION_EPSILON:
            if start < minimum or start > maximum:
                return None
            continue
        first = (minimum - start) / delta
        second = (maximum - start) / delta
        if first > second:
            first, second = second, first
        entry = max(entry, first)
        exit_parameter = min(exit_parameter, second)
        if entry > exit_parameter:
            return None
    if exit_parameter < 0.0 or entry > 1.0:
        return None
    return max(0.0, entry)


def _cylinder_entry_parameter(origin, target, shape):
    delta_x = target[0] - origin[0]
    delta_y = target[1] - origin[1]
    offset_x = origin[0] - shape['x']
    offset_y = origin[1] - shape['y']
    quadratic = delta_x * delta_x + delta_y * delta_y
    if quadratic <= _INTERSECTION_EPSILON:
        return None
    linear = 2.0 * (offset_x * delta_x + offset_y * delta_y)
    constant = offset_x * offset_x + offset_y * offset_y - shape['radius'] ** 2
    discriminant = linear * linear - 4.0 * quadratic * constant
    if discriminant < 0.0:
        return None
    root = math.sqrt(max(0.0, discriminant))
    first = (-linear - root) / (2.0 * quadratic)
    second = (-linear + root) / (2.0 * quadratic)
    if second < 0.0 or first > 1.0:
        return None
    return max(0.0, first)


def _entry_parameter(origin, target, shape):
    if shape['kind'] == 'box':
        return _box_entry_parameter(origin, target, shape)
    return _cylinder_entry_parameter(origin, target, shape)


def visible_boundary_points(shape, geometry, route, *, boundary_spacing, route_spacing):
    """Return collision samples directly visible from the frozen route."""
    observers = _sample_route(route, route_spacing)
    visible = []
    for point in boundary_points(shape, boundary_spacing):
        for observer in observers:
            if all(
                (entry := _entry_parameter(observer, point, obstacle)) is None
                or entry >= 1.0 - _INTERSECTION_EPSILON
                for obstacle in geometry
            ):
                visible.append(point)
                break
    return tuple(visible)


def _clearance(shape, x, y):
    if shape['kind'] == 'box':
        delta_x = max(abs(x - shape['x']) - shape['size_x'] / 2.0, 0.0)
        delta_y = max(abs(y - shape['y']) - shape['size_y'] / 2.0, 0.0)
        return math.hypot(delta_x, delta_y)
    return max(0.0, math.hypot(x - shape['x'], y - shape['y']) - shape['radius'])


def _map_local(grid, map_x, map_y):
    origin = grid['origin']
    delta_x = map_x - origin['x']
    delta_y = map_y - origin['y']
    cosine = math.cos(origin['yaw'])
    sine = math.sin(origin['yaw'])
    return (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
    )


def _map_cell(grid, map_x, map_y):
    local_x, local_y = _map_local(grid, map_x, map_y)
    column = math.floor(local_x / grid['resolution'])
    row = math.floor(local_y / grid['resolution'])
    if not (0 <= column < grid['width'] and 0 <= row < grid['height']):
        return None
    return row, column


def _world_to_map(transform, x, y):
    cosine = math.cos(transform['yaw'])
    sine = math.sin(transform['yaw'])
    return (
        transform['x'] + cosine * x - sine * y,
        transform['y'] + sine * x + cosine * y,
    )


def _occupied_near(grid, map_x, map_y, policy):
    resolution = grid['resolution']
    search_radius = policy['boundary_search_radius']
    local_x, local_y = _map_local(grid, map_x, map_y)
    minimum_column = math.floor((local_x - search_radius) / resolution)
    maximum_column = math.floor((local_x + search_radius) / resolution)
    minimum_row = math.floor((local_y - search_radius) / resolution)
    maximum_row = math.floor((local_y + search_radius) / resolution)
    for row in range(minimum_row, maximum_row + 1):
        for column in range(minimum_column, maximum_column + 1):
            if not (0 <= row < grid['height'] and 0 <= column < grid['width']):
                continue
            cell_min_x = column * resolution
            cell_max_x = cell_min_x + resolution
            cell_min_y = row * resolution
            cell_max_y = cell_min_y + resolution
            distance_x = max(cell_min_x - local_x, 0.0, local_x - cell_max_x)
            distance_y = max(cell_min_y - local_y, 0.0, local_y - cell_max_y)
            if math.hypot(distance_x, distance_y) > search_radius:
                continue
            value = grid['data'][row * grid['width'] + column]
            if value > policy['occupied_threshold']:
                return True
    return False


def evaluate_mapping_artifact(artifact):
    """Replay the frozen occupancy-quality oracle from JSON-safe evidence."""
    if artifact.get('schema_version') != 1:
        raise ValueError('unsupported Mapping artifact schema')
    policy = artifact['policy']
    grid = artifact['grid']
    transform = artifact['map_from_odom']
    geometry = tuple(artifact['geometry'])
    route = tuple(tuple(point) for point in artifact['route'])
    expected_cells = int(grid['width']) * int(grid['height'])
    if len(grid['data']) != expected_cells:
        raise ValueError('Mapping artifact grid data length does not match dimensions')
    if abs(grid['resolution'] - policy['map_resolution']) > 1.0e-9:
        raise ValueError('Mapping artifact resolution violates its quality policy')

    known = 0
    navigable = 0
    coordinate = policy['floor_min'] + policy['minimum_route_clearance']
    upper = policy['floor_max'] - policy['minimum_route_clearance']
    while coordinate <= upper + 1.0e-9:
        other = policy['floor_min'] + policy['minimum_route_clearance']
        while other <= upper + 1.0e-9:
            if (
                min(_clearance(shape, coordinate, other) for shape in geometry)
                >= policy['minimum_route_clearance']
            ):
                navigable += 1
                map_x, map_y = _world_to_map(transform, coordinate, other)
                cell = _map_cell(grid, map_x, map_y)
                if cell is not None:
                    row, column = cell
                    if grid['data'][row * grid['width'] + column] >= 0:
                        known += 1
            other += policy['map_resolution']
        coordinate += policy['map_resolution']
    known_ratio = known / navigable if navigable else 0.0

    boundary_ratios = {}
    for shape in geometry:
        points = visible_boundary_points(
            shape,
            geometry,
            route,
            boundary_spacing=policy['map_resolution'],
            route_spacing=policy['map_resolution'],
        )
        if not points:
            raise ValueError(f'Mapping artifact has no visible boundary for {shape["name"]}')
        occupied = 0
        for x, y in points:
            map_x, map_y = _world_to_map(transform, x, y)
            if _occupied_near(grid, map_x, map_y, policy):
                occupied += 1
        boundary_ratios[shape['name']] = occupied / len(points)
    return {
        'known_floor_ratio': known_ratio,
        'boundary_ratios': boundary_ratios,
        'minimum_boundary_ratio': min(boundary_ratios.values()),
        'navigable_samples': navigable,
    }
