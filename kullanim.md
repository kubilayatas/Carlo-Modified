# CARLO Çizgi İzleyen Labirent Simülatörü - Kullanım Kılavuzu

Bu belge, Çizgi İzleyen Robot ve Labirent Çözücü simülasyonunu nasıl kullanacağınızı, parametreleri nasıl değiştireceğinizi ve kendi rotalarınızı / algoritmalarınızı nasıl geliştireceğinizi açıklar.

## 1. Simülasyonu Çalıştırmak
Ana demonun çalıştırılması için terminalde proje dizinindeyken şu komutu girin:
```bash
python example_line_maze.py
```
Simülasyon açıldığında seçili labirent planına göre robot başlangıç pozisyonundan sensörleri ile vizyon sağlayarak hedefe ulaşmaya çalışacaktır.

---

## 2. Labirenti Değiştirmek
Farklı zorluklardaki labirentlerde test yapmak isterseniz `example_line_maze.py` dosyasını düzenleyin:

```python
# Dosyanın başlarındaki import kısmına grid adını ekleyin
from maze_presets import MAZE_MEDIUM_T, MAZE_EXPERT, MAZE_MineOne

# Kullanılacak matrisi belirtin:
maze_grid = MAZE_MineOne
```
Kendi labirentinizi eklemek isterseniz `maze_presets.py` dosyasına girerek yeni bir `Numpy Array` oluşturabilirsiniz.  
Kullanılan Sayılar: `0: Boş/Duvar`, `1: Yol`, `2: Başlangıç`, `3: Bitiş`.

---

## 3. Parametrelerin Anlamları ve Modifikasyon

Konfigürasyonunuzu yapabileceğiniz tüm kritik parametreler şunlardır:

### PID Kontrolcü (`pid = PIDController(...)`)
- **`kp` (Proportional)**: Robotun çizgiden saptığında vereceği sert tepkiyi belirler. Çok düşükse viraj dönülemez, çok yüksekse araç titreyerek/ziplayarak yola devam eder. (Örn: `2.5` - `10.0`)
- **`ki` (Integral)**: Zamanla biriken kalıcı hataları düzeltir. Çizgi izleyicide genelde çok düşük tutulur (`0.1`, `0.01`).
- **`kd` (Derivative)**: Aracın sönümlemesidir. KP'nin yarattığı agresif sallanmayı azaltarak yola pürüzsüz oturmasını sağlar. (`0.15` - `0.5` bandı).

### Sürüş ve Hız (`PathExecutor(...)`)
- **`base_speed`**: Robotun en yüksek otonom düz yol hızıdır (m/s). Robot bir viraj tespit edip çizgiden hafif sağa/sola taşarsa bu hızı otomatik düşürecek şekilde kodlanmıştır.
- **`junction_threshold`**: Bir kavşağa ulaşıldığının tescillenmesi için gereken maksimum mesafe toleransıdır.

### Dünya Parametreleri
- **`cell_size`**: Labirentin tek bir karesinin fiziksel uzunluğudur (Örn: `0.2` = 20 cm).
- **`dt`**: Simülasyon döngüsü zaman adımı (`0.1`).
- **`ppm`**: Ekrana yansıtılan 1 metrenin kaç piksel ile çizileceğini belirtir (`300`). Ekranda küçük/büyük gelirse değiştirebilirsiniz.

### Araç ve Sensörler (`LineFollowerRobot(...)`)
- **`size`**: Aracın varsayılan fiziksel hit-box'ıdır. Örn: `Point(0.15, 0.08)` -> 15cm boy, 8cm en.
- **`sensor_count`**: Robotun önündeki IR sensör adedidir.
- **`sensor_spread`**: Algılayıcıların kapsadığı genellik aralığı.
- **`sensor_forward_offset`**: Sensörlerin robotun merkezinden ne kadar önde olacağı.

---

## 4. Kendi Algoritmanızı Eklemek (Path Planning)
Gömülü BFS/Küme algoritmaları yerine kendi yapay zeka/hesaplama mantığınızı kurmak için:

1. **`maze_solver.py`** dosyasını açıp `MazePlanner` sınıfına fonksiyonunuzu ekleyin:
```python
class MazePlanner:
    # ...
    def benim_algoritmam(self, start: tuple, end: tuple) -> list:
        path = [start]
        # self.adj (Sözlük) ve self.maze.grid (Matris) kullanarak hesapla...
        # path.append(hedef_dugum) diyerek rotanızı çıkarın.
        return path
```

2. **`example_line_maze.py`** içerisinde bu fonksiyonu tetikleyin:
```python
# ---- Yol planla ----
planner = MazePlanner(maze)
path = planner.benim_algoritmam(start_cell, end_cell)
```

Robot artık sizin algoritmanızın ürettiği patikayı otonom bir şekilde PID kullanarak takip edecektir.
