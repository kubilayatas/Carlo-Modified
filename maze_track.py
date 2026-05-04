import numpy as np
from agents import Painting
from geometry import Point

class MazeTrack:
    def __init__(self, grid, cell_size: float = 0.3, line_width: float = 0.03,
                 line_color: str = 'black', origin_x: float = None, origin_y: float = None):
        """
        grid: 2D liste veya numpy array. 0=boş, 1=yol, 2=başlangıç, 3=bitiş.
        cell_size: Bir hücrenin metre boyutu.
        line_width: Çizgi genişliği (metre).
        origin_x, origin_y: Labirentin sol-alt köşesi (dünya koordinatı).
                            None ise labirent dünya ortasına yerleştirilir.
        """
        self.grid = np.array(grid)
        self.rows, self.cols = self.grid.shape
        self.cell_size = cell_size
        self.line_width = line_width
        self.line_color = line_color

        # Otomatik ortalama
        maze_width = self.cols * cell_size
        maze_height = self.rows * cell_size
        if origin_x is None:
            # World genişliği bilinmediği için çağıran taraf verebilir
            # ya da varsayılan olarak (0,0) kullanılır
            origin_x = 0.0
        if origin_y is None:
            origin_y = 0.0
        self.origin_x = origin_x
        self.origin_y = origin_y

    def cell_to_world(self, r: int, c: int) -> Point:
        """Grid hücre (r, c) → dünya koordinatı (merkez noktası)."""
        wx = self.origin_x + c * self.cell_size + self.cell_size / 2.0
        wy = self.origin_y + (self.rows - 1 - r) * self.cell_size + self.cell_size / 2.0
        return Point(wx, wy)

    def world_to_cell(self, x: float, y: float) -> tuple:
        """Dünya koordinatı → en yakın grid hücresi (r, c).
        Sensör modunda robotun hangi hücrede olduğunu bulmak için kullanılır."""
        c = round((x - self.origin_x - self.cell_size / 2.0) / self.cell_size)
        r = self.rows - 1 - round((y - self.origin_y - self.cell_size / 2.0) / self.cell_size)
        r = max(0, min(self.rows - 1, r))
        c = max(0, min(self.cols - 1, c))
        return (r, c)

    def get_start_position(self):
        """Grid'de 2 ile işaretlenmiş hücrenin dünya koordinatını döndür."""
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == 2:
                    return self.cell_to_world(r, c), (r, c)
        raise ValueError("Grid'de başlangıç noktası (2) bulunamadı!")

    def get_end_position(self):
        """Grid'de 3 ile işaretlenmiş hücrenin dünya koordinatını döndür."""
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == 3:
                    return self.cell_to_world(r, c), (r, c)
        raise ValueError("Grid'de bitiş noktası (3) bulunamadı!")

    def build_paintings(self) -> list:
        """Labirent çizgilerini Painting nesneleri olarak üret."""
        paintings = []
        lw = self.line_width

        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == 0:
                    continue

                # Hücre merkez patch'i
                center = self.cell_to_world(r, c)
                node_color = self.line_color
                if self.grid[r][c] == 2:
                    node_color = 'green'  # Başlangıç
                elif self.grid[r][c] == 3:
                    node_color = 'red'    # Bitiş

                paintings.append(
                    Painting(center, Point(lw, lw), node_color)
                )

                # Sağ komşuyla bağlantı (yatay)
                if c + 1 < self.cols and self.grid[r][c + 1] > 0:
                    mid_x = (self.cell_to_world(r, c).x + self.cell_to_world(r, c + 1).x) / 2.0
                    mid_y = center.y
                    seg_len = self.cell_size  # iki merkez arası tam cell_size
                    paintings.append(
                        Painting(Point(mid_x, mid_y), Point(seg_len, lw), self.line_color)
                    )

                # Alt komşuyla bağlantı (dikey) — grid'de r+1 aşağı satır = dünyada daha düşük y
                if r + 1 < self.rows and self.grid[r + 1][c] > 0:
                    mid_y2 = (self.cell_to_world(r, c).y + self.cell_to_world(r + 1, c).y) / 2.0
                    mid_x2 = center.x
                    seg_len = self.cell_size
                    paintings.append(
                        Painting(Point(mid_x2, mid_y2), Point(lw, seg_len), self.line_color)
                    )

        return paintings

    def add_to_world(self, world):
        """Tüm Painting nesnelerini world'e ekle."""
        paintings = self.build_paintings()
        for p in paintings:
            world.add(p)
        return paintings

    def get_adjacency(self):
        """Grid'in graf gösterimini çıkar. Labirent çözme algoritmaları için.
        Dönüş: dict { (r,c): [(r2,c2), ...] } — komşuluk listesi.
        Sadece grid değeri > 0 olan hücreler dahil."""
        adj = {}
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == 0:
                    continue
                neighbors = []
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < self.rows and 0 <= nc < self.cols and self.grid[nr][nc] > 0:
                        neighbors.append((nr, nc))
                adj[(r, c)] = neighbors
        return adj

    def get_topology_at(self, r: int, c: int):
        """Bir hücredeki topolojiyi analiz et.
        Dönüş: dict { 'up': bool, 'down': bool, 'left': bool, 'right': bool, 'type': str }
        type: 'dead_end', 'straight', 'turn', 'T', 'cross'
        """
        if self.grid[r][c] == 0:
            return None
        dirs = {
            'up':    (r - 1, c),
            'down':  (r + 1, c),
            'left':  (r, c - 1),
            'right': (r, c + 1)
        }
        result = {}
        count = 0
        for name, (nr, nc) in dirs.items():
            has = (0 <= nr < self.rows and 0 <= nc < self.cols and self.grid[nr][nc] > 0)
            result[name] = has
            if has:
                count += 1

        if count <= 1:
            result['type'] = 'dead_end'
        elif count == 2:
            if (result['up'] and result['down']) or (result['left'] and result['right']):
                result['type'] = 'straight'
            else:
                result['type'] = 'turn'
        elif count == 3:
            result['type'] = 'T'
        else:
            result['type'] = 'cross'

        return result
