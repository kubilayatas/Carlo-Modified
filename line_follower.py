from entities import DiffDriveEntity
from geometry import Point
import numpy as np

class LineFollowerRobot(DiffDriveEntity):
    def __init__(self, center: Point, heading: float,
                 color: str = 'green3',
                 sensor_count: int = 8,
                 sensor_spread: float = 0.056,
                 sensor_forward_offset: float = 0.075):
        size = Point(0.15, 0.08)
        movable = True
        friction = 0.05
        super().__init__(center, heading, size, movable, friction)

        self.color = color
        self.collidable = True
        self.max_speed = 0.8
        self.min_speed = 0.0

        # Sensör konfigürasyonu
        self.sensor_count = sensor_count
        self.sensor_spread = sensor_spread
        self.sensor_forward_offset = sensor_forward_offset

        # Sensör pozisyonları (yerel koordinat, robot merkezine göre)
        self._local_sensor_positions = self._compute_local_sensor_positions()

        # Sensör okumaları: True = çizgi üzerinde (beyaz), False = zemin (siyah)
        self.sensor_readings = [False] * sensor_count

        # Dünya referansı (sensör okuması için Painting nesnelerine erişim gerekli)
        self._world_ref = None

    def _compute_local_sensor_positions(self):
        """Sensörlerin robot yerel koordinatlarındaki (dx, dy) offsetlerini hesapla.
        Robot heading=0 iken x ekseni ileri yönü temsil eder."""
        positions = []
        half_spread = self.sensor_spread / 2.0
        for i in range(self.sensor_count):
            if self.sensor_count == 1:
                lateral = 0.0
            else:
                lateral = -half_spread + i * self.sensor_spread / (self.sensor_count - 1)
            # dx = ileri (heading yönünde), dy = yana (heading'e dik)
            positions.append((self.sensor_forward_offset, lateral))
        return positions

    def attach_to_world(self, world):
        """World referansını bağla. World.add() çağrıldıktan sonra çağrılmalı."""
        self._world_ref = world

    def get_sensor_world_positions(self):
        """Sensörlerin dünya koordinatlarını hesapla (heading dönüşümü uygula)."""
        cos_h = np.cos(self.heading)
        sin_h = np.sin(self.heading)
        positions = []
        for (dx, dy) in self._local_sensor_positions:
            # 2D rotasyon: heading açısına göre döndür
            wx = self.center.x + dx * cos_h - dy * sin_h
            wy = self.center.y + dx * sin_h + dy * cos_h
            positions.append(Point(wx, wy))
        return positions

    def update_sensors(self):
        """Her tick'te çağrılır. Sensörlerin çizgi üzerinde olup olmadığını kontrol et."""
        if self._world_ref is None:
            return

        sensor_positions = self.get_sensor_world_positions()
        self.sensor_readings = []

        for sp in sensor_positions:
            on_line = False
            for agent in self._world_ref.static_agents:
                # Sadece Painting (collidable=False) nesneleri çizgidir (Zemin hariç)
                if not agent.collidable and hasattr(agent, 'obj'):
                    if getattr(agent, 'color', '') in ['black', 'green', 'red']:
                        if sp.isInside(agent.obj):
                            on_line = True
                            break
            self.sensor_readings.append(on_line)

    def tick(self, dt: float):
        """Üst sınıfın tick'ini çağır, sonra sensörleri güncelle."""
        super().tick(dt)
        self.update_sensors()

    def get_line_error(self):
        """PID kontrolcü için hata değeri hesapla.
        Dönüş: -1.0 (çizgi tamamen solda) ile +1.0 (çizgi tamamen sağda) arasında.
        Çizgi ortadaysa 0.0. Çizgi kayıpsa None döner."""
        if not any(self.sensor_readings):
            return None  # Çizgi kayıp

        n = self.sensor_count
        # Ağırlıklı ortalama: her sensöre [-1, +1] aralığında bir konum ata
        weights = []
        for i in range(n):
            if n == 1:
                pos = 0.0
            else:
                pos = -1.0 + 2.0 * i / (n - 1)
            weights.append(pos)

        weighted_sum = sum(w * (1.0 if r else 0.0) for w, r in zip(weights, self.sensor_readings))
        active_count = sum(1 for r in self.sensor_readings if r)

        if active_count == 0:
            return None

        return weighted_sum / active_count

    @property
    def sensor_debug_str(self):
        """Debug çıktısı: - = zemin, X = çizgi"""
        return ''.join('X' if r else '-' for r in self.sensor_readings)

    @property
    def active_sensor_count(self):
        """Aktif (çizgi üzerinde) sensör sayısı."""
        return sum(1 for r in self.sensor_readings if r)

    def detect_junction(self, threshold: int = None) -> str:
        """Sensör pattern'ine bakarak kavşak durumunu belirle.

        Args:
            threshold: Kaç sensör aktif olunca kavşak sayılacak.
                       None ise sensor_count - 2 kullanılır (8 sensör → 6).

        Dönüş:
            'junction' — T veya 4-yol kavşak (çoğu sensör aktif)
            'lost'     — Çizgi tamamen kayıp (çıkmaz sokak veya sapma)
            'line'     — Normal çizgi izleme durumu
        """
        if threshold is None:
            threshold = max(self.sensor_count - 2, 3)
        active = self.active_sensor_count
        if active >= threshold:
            return 'junction'
        elif active == 0:
            return 'lost'
        else:
            return 'line'
