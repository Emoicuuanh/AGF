import random

def merge_zones_to_single(zones_in):
    """
    Merge all zones into a single entry with:
    - type_zone: a set of unique type_zone values.
    - properties: merged properties where conflicting values are chosen randomly.
    """
    merged_type_zone = set()
    merged_properties = {}

    for zone in zones_in:
        # Gộp type_zone
        merged_type_zone.add(zone['type_zone'])

        # Gộp properties
        for key, value in zone['properties'].items():
            if key in merged_properties:
                # Nếu key đã tồn tại, chọn ngẫu nhiên giữa giá trị hiện tại và giá trị mới
                merged_properties[key] = random.choice([merged_properties[key], value])
            else:
                # Nếu key chưa tồn tại, thêm vào
                merged_properties[key] = value

    # Kết quả cuối cùng
    return [{'type_zone': merged_type_zone, 'properties': merged_properties}]

# Ví dụ sử dụng
zones_in = [
    {'type_zone': 'RESTRICTED_ZONE', 'properties': {'Safety': 'Enable', 'Led': 'Enable', 'Sound': 'Enable', 'Velocy': 0.5}},
    {'type_zone': 'CHARGE_ZONE', 'properties': {'Safety': 'Disable', 'Led': 'Enable', 'Sound': 'Disable', 'Velocy': 0.2}},
    {'type_zone': 'RESTRICTED_ZONE', 'properties': {'Safety': 'Disable', 'Sound': 'Enable', 'Velocy': 0.5}},
    {'type_zone': 'CHARGE_ZONE', 'properties': {'Safety': 'Enable', 'Led': 'Disable', 'Sound': 'Disable', 'Velocy': 0.3}},
]

merged_zones = merge_zones_to_single(zones_in)
print(merged_zones)
