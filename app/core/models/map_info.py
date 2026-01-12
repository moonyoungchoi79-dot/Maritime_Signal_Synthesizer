from dataclasses import dataclass

@dataclass
class MapInfo:
    center_lat: float = 35.0
    center_lon: float = 129.0
    pixels_per_degree: float = 2000.0 
    
    utm_zone: int = 52
    utm_zone_letter: str = 'S'
    utm_hemisphere: str = 'N'
    
    def update_utm_from_latlon(self):
        pass
