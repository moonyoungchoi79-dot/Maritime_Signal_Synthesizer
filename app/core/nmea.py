import random

def calculate_checksum(nmea_str):
    calc_cksum = 0
    for s in nmea_str:
        calc_cksum ^= ord(s)
    return hex(calc_cksum)[2:].upper().zfill(2)

def encode_ais_payload(mmsi):
    # Simplified AIS payload that includes MMSI
    # A real AIS payload encoding is more complex (6-bit ASCII, specific fields)
    # For demonstration, we'll embed the MMSI and pad with dummy chars.
    
    mmsi_str = str(mmsi).zfill(9) # Ensure MMSI is 9 digits
    dummy_payload_start = f"MMSI:{mmsi_str}"
    
    # Pad to 28 characters with random characters if needed
    chars = "0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVW`abcdefghijklmnopqrstuvw"
    remaining_len = 28 - len(dummy_payload_start)
    if remaining_len > 0:
        padding = "".join(random.choices(chars, k=remaining_len))
    else:
        padding = ""
        
    return (dummy_payload_start + padding)[:28] # Ensure total length is 28

def parse_nmea_fields(raw):
    fields = {}
    raw_str = raw.strip()
    if not raw_str.startswith("$") and not raw_str.startswith("!"): return fields
    
    parts = raw_str.split('*')[0].split(',')
    
    header = parts[0]
    if header.startswith('$') or header.startswith('!'):
        header = header[1:]
    
    fields['talker'] = header[:2]
    fields['sentence_type'] = header[2:]
    fields['raw'] = raw_str
    
    stype = fields['sentence_type']
    
    try:
        if stype == 'RMC':
            if len(parts) > 9:
                fields['lat_deg'] = _parse_lat(parts[3], parts[4])
                fields['lon_deg'] = _parse_lon(parts[5], parts[6])
                fields['sog_knots'] = float(parts[7]) if parts[7] else 0.0
                fields['cog_true_deg'] = float(parts[8]) if parts[8] else 0.0
        elif stype == 'GGA':
            if len(parts) > 5:
                fields['lat_deg'] = _parse_lat(parts[2], parts[3])
                fields['lon_deg'] = _parse_lon(parts[4], parts[5])
        elif stype == 'VTG':
            if len(parts) > 7:
                fields['cog_true_deg'] = float(parts[1]) if parts[1] else 0.0
                fields['sog_knots'] = float(parts[5]) if parts[5] else 0.0
        elif stype == 'GLL':
            if len(parts) > 4:
                fields['lat_deg'] = _parse_lat(parts[1], parts[2])
                fields['lon_deg'] = _parse_lon(parts[3], parts[4])
        elif stype == 'HDT':
            if len(parts) > 1:
                fields['heading_true_deg'] = float(parts[1]) if parts[1] else 0.0
        elif stype == 'TLL':
            if len(parts) > 5:
                try: fields['target_no'] = int(parts[1])
                except: pass
                fields['lat_deg'] = _parse_lat(parts[2], parts[3])
                fields['lon_deg'] = _parse_lon(parts[4], parts[5])
        elif stype == 'TTM':
             if len(parts) > 1:
                try: fields['target_no'] = int(parts[1])
                except: pass
                 
        if 'lat_deg' in fields and 'lon_deg' in fields:
             fields['utm_easting_m'] = 0.0
             fields['utm_northing_m'] = 0.0
             fields['utm_zone'] = 0
             fields['utm_hemisphere'] = 'N'
            
    except:
        pass
    
    return fields

def _parse_lat(val, ns):
    if not val: return 0.0
    try:
        d = float(val[:2])
        m = float(val[2:])
        v = d + m/60.0
        return v if ns == 'N' else -v
    except: return 0.0

def _parse_lon(val, ew):
    if not val: return 0.0
    try:
        d = float(val[:3])
        m = float(val[3:])
        v = d + m/60.0
        return v if ew == 'E' else -v
    except: return 0.0