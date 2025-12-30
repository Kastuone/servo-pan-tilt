from ultralytics import YOLO
import cv2
import numpy as np
import requests
import time
import math

# YOLO modelini yükle
model = YOLO("models/best.pt")

class PanTiltController:
    def __init__(self, esp32_ip="192.168.43.185"):
        self.esp32_ip = esp32_ip
        self.camera = None
        self.running = False
        self.bullseye_tracking = False  # Bullseye takibi modu
        
        # Kamera çözünürlüğü
        self.frame_width = 1280
        self.frame_height = 720
        
        # Hassas servo kontrolü için float pozisyonlar - YENİ MERKEZ
        self.current_pan = 114.0  # Yeni merkez pan değeri
        self.current_tilt = 14.0  # Yeni merkez tilt değeri
        
        # Mikrosaniye değerleri (MG995 için)
        self.SERVO_MIN_US = 1000
        self.SERVO_MAX_US = 2000
        self.SERVO_CENTER_US = 1500
        
        # Mevcut mikrosaniye pozisyonları
        self.current_pan_us = self.degrees_to_microseconds(self.current_pan)
        self.current_tilt_us = self.degrees_to_microseconds(self.current_tilt)
        
        # Manuel kontrol için hassas adım sistemi
        self.step_size = 0.5  # Varsayılan 0.5 derece
        self.step_min = 0.1   # Minimum 0.1 derece
        self.step_max = 10.0  # Maksimum 10 derece
        self.step_increment = 0.1
        
        # Mikrosaniye adım modu
        self.use_micros_mode = False  # Mikrosaniye modunu aç/kapa
        self.micros_step = 10  # Mikrosaniye adımı
        
        # Zoom kontrol parametreleri
        self.zoom_level = 1.0
        self.zoom_min = 1.0
        self.zoom_max = 5.0
        self.zoom_step = 0.1
        
        # YOLO model parametreleri
        self.confidence_threshold = 0.5
        self.bullseye_class_name = "bullseye"
        
        # Hedef kilitleme sistemi
        self.target_locked = False
        self.target_box = None  # Hedef bullseye'ın kutusu
        self.dead_zone_size = 20  # SABİT 20 piksel dead zone
        
        # Zoom kontrol parametreleri - YENİ
        self.zoom_in_threshold = 0.55   # Targetbox deadbox'un 0.7 katından küçükse zoom in
        self.zoom_out_threshold = 0.302  # Targetbox deadbox'un 0.35 katından küçükse zoom out
        
        # Bullseye takibi için kontrol parametreleri
        self.last_bullseye_move_time = 0
        self.bullseye_move_interval = 0.2  # Hareket aralığı
        
        # Kayıp hedef takip sistemi - YENİ
        self.last_bullseye_detection_time = time.time()
        self.last_known_target_center = None  # Son bilinen hedef merkezi
        self.target_lost_time = None  # Hedefin kaybolduğu zaman
        self.continue_tracking_duration = 3.0  # 3 saniye boyunca son yöne bakmaya devam et
        self.no_bullseye_timeout = 5.0
        self.lost_target_recovery = False
        
        # Servo sınırları - Güncellenmiş
        self.pan_min = 30
        self.pan_max = 290
        self.tilt_min = 0
        self.tilt_max = 180  # Tilt sınırları 0-180

    def degrees_to_microseconds(self, degrees):
        """Dereceyi mikrosaniyeye çevir"""
        degrees = max(0, min(180, degrees))
        return int(self.SERVO_MIN_US + (degrees / 180.0) * (self.SERVO_MAX_US - self.SERVO_MIN_US))
    
    def microseconds_to_degrees(self, microseconds):
        """Mikrosaniyeyi dereceye çevir"""
        microseconds = max(self.SERVO_MIN_US, min(self.SERVO_MAX_US, microseconds))
        return ((microseconds - self.SERVO_MIN_US) / (self.SERVO_MAX_US - self.SERVO_MIN_US)) * 180.0

    def initialize_camera(self, camera_index=1):
        """Kamerayı başlat"""
        self.camera = cv2.VideoCapture(camera_index)
        if not self.camera.isOpened():
            print(f"Kamera {camera_index} açılamadı!")
            return False
            
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        print("Kamera başlatıldı")
        return True
    
    def apply_zoom(self, frame):
        """Yazılımsal zoom uygula"""
        if self.zoom_level <= 1.0:
            return frame
        
        h, w = frame.shape[:2]
        new_w = int(w / self.zoom_level)
        new_h = int(h / self.zoom_level)
        
        start_x = (w - new_w) // 2
        start_y = (h - new_h) // 2
        
        cropped = frame[start_y:start_y + new_h, start_x:start_x + new_w]
        zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)
        
        return zoomed
    
    def send_servo_command(self, pan=None, tilt=None, use_micros=False):
        """ESP32'ye hassas servo komutları gönder"""
        try:
            if pan is not None and tilt is not None:
                if use_micros:
                    # Mikrosaniye modunda gönder
                    pan_us = int(pan)
                    tilt_us = int(tilt)
                    
                    # Sınırları kontrol et
                    pan_us = max(self.SERVO_MIN_US, min(self.SERVO_MAX_US, pan_us))
                    tilt_us = max(self.SERVO_MIN_US, min(self.SERVO_MAX_US, tilt_us))
                    
                    url = f"http://{self.esp32_ip}/control_micros"
                    data = {"pan_us": pan_us, "tilt_us": tilt_us}
                    
                    # Pozisyonları güncelle
                    self.current_pan_us = pan_us
                    self.current_tilt_us = tilt_us
                    self.current_pan = self.microseconds_to_degrees(pan_us)
                    self.current_tilt = self.microseconds_to_degrees(tilt_us)
                    
                    print(f"📡 Mikrosaniye gönderiliyor - Pan: {pan_us}μs ({self.current_pan:.2f}°) | Tilt: {tilt_us}μs ({self.current_tilt:.2f}°)")
                    
                else:
                    # Float derece modunda gönder
                    pan = float(pan)
                    tilt = float(tilt)
                    
                    # Sınırları kontrol et
                    pan = max(0, min(180, pan))
                    tilt = max(self.tilt_min, min(self.tilt_max, tilt))  # Tilt için güncellenen sınır
                    
                    url = f"http://{self.esp32_ip}/control"
                    data = {"pan": f"{pan:.2f}", "tilt": f"{tilt:.2f}"}
                    
                    # Pozisyonları güncelle
                    self.current_pan = pan
                    self.current_tilt = tilt
                    self.current_pan_us = self.degrees_to_microseconds(pan)
                    self.current_tilt_us = self.degrees_to_microseconds(tilt)
                    
                    print(f"📡 Derece gönderiliyor - Pan: {pan:.2f}° ({self.current_pan_us}μs) | Tilt: {tilt:.2f}° ({self.current_tilt_us}μs)")
                
                response = requests.post(url, data=data, timeout=2)
                
            else:
                url = f"http://{self.esp32_ip}/status"
                response = requests.get(url, timeout=2)
            
            if response.status_code == 200:
                result = response.json()
                # Gelen verileri güncelle
                if 'pan' in result:
                    self.current_pan = float(result['pan'])
                if 'tilt' in result:
                    self.current_tilt = float(result['tilt'])
                if 'pan_us' in result:
                    self.current_pan_us = int(result['pan_us'])
                if 'tilt_us' in result:
                    self.current_tilt_us = int(result['tilt_us'])
                return result
            else:
                print(f"ESP32 yanıt hatası: {response.status_code}")
                return None
                
        except requests.exceptions.RequestException as e:
            print(f"ESP32 bağlantı hatası: {e}")
            return None
    
    def manual_move(self, direction):
        """WASD ile hassas manuel hareket"""
        if self.use_micros_mode:
            # Mikrosaniye modunda hareket
            if direction == 'w':  # Yukarı
                self.current_tilt_us = max(self.SERVO_MIN_US, self.current_tilt_us - self.micros_step)
            elif direction == 's':  # Aşağı
                self.current_tilt_us = min(self.SERVO_MAX_US, self.current_tilt_us + self.micros_step)
            elif direction == 'a':  # Sol
                self.current_pan_us = max(self.SERVO_MIN_US, self.current_pan_us + self.micros_step)
            elif direction == 'd':  # Sağ
                self.current_pan_us = min(self.SERVO_MAX_US, self.current_pan_us - self.micros_step)
            
            self.send_servo_command(self.current_pan_us, self.current_tilt_us, use_micros=True)
            print(f"🎮 Mikrosaniye hareketi: Pan={self.current_pan_us}μs, Tilt={self.current_tilt_us}μs, Adım={self.micros_step}μs")
            
        else:
            # Float derece modunda hassas hareket
            if direction == 'w':  # Yukarı
                self.current_tilt = max(self.tilt_min, self.current_tilt - self.step_size)
            elif direction == 's':  # Aşağı
                self.current_tilt = min(self.tilt_max, self.current_tilt + self.step_size)
            elif direction == 'a':  # Sol
                self.current_pan = max(self.pan_min, self.current_pan + self.step_size)
            elif direction == 'd':  # Sağ
                self.current_pan = min(self.pan_max, self.current_pan - self.step_size)
            
            self.send_servo_command(self.current_pan, self.current_tilt, use_micros=False)
            print(f"🎮 Manuel hareket: Pan={self.current_pan:.2f}°, Tilt={self.current_tilt:.2f}°, Adım={self.step_size:.2f}°")
    
    def adjust_step_size(self, increase=True):
        """Hassas adım boyutunu ayarla"""
        if self.use_micros_mode:
            if increase:
                self.micros_step = min(100, self.micros_step + 5)
            else:
                self.micros_step = max(1, self.micros_step - 5)
            print(f"⚡ Mikrosaniye adımı: {self.micros_step}μs")
        else:
            if increase:
                self.step_size = min(self.step_max, self.step_size + self.step_increment)
            else:
                self.step_size = max(self.step_min, self.step_size - self.step_increment)
            print(f"📏 Derece adımı: {self.step_size:.2f}°")
    
    def toggle_micros_mode(self):
        """Mikrosaniye/derece modu geçişi"""
        self.use_micros_mode = not self.use_micros_mode
        mode = "MİKROSANİYE" if self.use_micros_mode else "DERECE"
        print(f"🔄 Kontrol modu: {mode}")
    
    def precise_position(self, pan_deg, tilt_deg):
        """Çok hassas pozisyonlama (0.1 derece hassasiyetle)"""
        print(f"🎯 Hassas pozisyonlama: Pan={pan_deg:.2f}°, Tilt={tilt_deg:.2f}°")
        self.send_servo_command(pan_deg, tilt_deg, use_micros=False)
    
    def is_dead_zone_inside_target(self, target_box):
        """Dead zone'un (dairesel) target box içinde olup olmadığını kontrol et"""
        if target_box is None:
            return False
        
        x, y, w, h = target_box
        
        frame_center_x = self.frame_width // 2
        frame_center_y = self.frame_height // 2
        
        # Dairesel dead zone'un tamamen target box içinde olması için
        # dairenin en uç noktalarının target box sınırları içinde olması gerekir
        target_left = x
        target_right = x + w
        target_top = y
        target_bottom = y + h
        
        # Dairesel dead zone'un çevresindeki 4 uç nokta
        left_point = frame_center_x - self.dead_zone_size
        right_point = frame_center_x + self.dead_zone_size
        top_point = frame_center_y - self.dead_zone_size
        bottom_point = frame_center_y + self.dead_zone_size
        
        return (left_point >= target_left and right_point <= target_right and
                top_point >= target_top and bottom_point <= target_bottom)
    
    def track_to_target_center(self, center_x, center_y):
        """Hedefi merkeze getir - ULTRA hassas takip"""
        frame_center_x = self.frame_width // 2
        frame_center_y = self.frame_height // 2
        
        diff_x = center_x - frame_center_x
        diff_y = center_y - frame_center_y
        
        # Piksel farkına göre hassas derece hesaplama
        base_sensitivity = 0.008  # MG995 için optimize edilmiş
        
        sensitivity = base_sensitivity / self.zoom_level
        
        distance = math.sqrt(diff_x**2 + diff_y**2)
        
        if distance < 20:  # Çok yakın - ultra hassas
            sensitivity *= 0.3
            step_multiplier = 0.2
        elif distance < 50:  # Yakın - hassas
            sensitivity *= 0.5
            step_multiplier = 0.5
        elif distance < 100:  # Orta
            sensitivity *= 0.8
            step_multiplier = 1.0
        else:  # Uzak - hızlı hareket
            sensitivity *= 1.2
            step_multiplier = 1.5
        
        pan_change = diff_x * sensitivity * step_multiplier
        tilt_change = diff_y * sensitivity * step_multiplier
        
        if abs(pan_change) < 0.05:
            pan_change = 0
        if abs(tilt_change) < 0.05:
            tilt_change = 0
        
        new_pan = self.current_pan - pan_change
        new_tilt = self.current_tilt + tilt_change
        
        new_pan = max(self.pan_min, min(self.pan_max, new_pan))
        new_tilt = max(self.tilt_min, min(self.tilt_max, new_tilt))
        
        self.send_servo_command(new_pan, new_tilt, use_micros=False)
        
        if abs(diff_x) > 5 or abs(diff_y) > 5:
            print(f"🎯 Takip: Δx={diff_x:+4d}px, Δy={diff_y:+4d}px | "
                  f"Pan: {self.current_pan:.2f}° ({pan_change:+.3f}°) | "
                  f"Tilt: {self.current_tilt:.2f}° ({tilt_change:+.3f}°)")
        
        return abs(diff_x) < 5 and abs(diff_y) < 5
    
    def smooth_move_to(self, target_pan, target_tilt, duration=2.0, steps=50):
        """Yumuşak hareket fonksiyonu - interpolasyon ile"""
        print(f"🌊 Yumuşak hareket başlıyor: {self.current_pan:.2f}° → {target_pan:.2f}°, "
              f"{self.current_tilt:.2f}° → {target_tilt:.2f}°")
        
        start_pan = self.current_pan
        start_tilt = self.current_tilt
        
        def ease_in_out(t):
            return 0.5 * (1 - math.cos(math.pi * t))
        
        for i in range(steps + 1):
            t = i / steps
            eased_t = ease_in_out(t)
            
            current_pan = start_pan + (target_pan - start_pan) * eased_t
            current_tilt = start_tilt + (target_tilt - start_tilt) * eased_t
            
            self.send_servo_command(current_pan, current_tilt, use_micros=False)
            
            time.sleep(duration / steps)
        
        print(f"✅ Yumuşak hareket tamamlandı: Pan={self.current_pan:.2f}°, Tilt={self.current_tilt:.2f}°")
    
    def calibrate_servo_range(self):
        """Servo aralığını kalibre et"""
        print("🔧 KALİBRASYON BAŞLIYOR...")
        print("Servolar test ediliyor. Lütfen bekleyin...")
        
        test_positions = [
            (500, "Minimum (0°)"),
            (1000, "45°"),
            (1500, "Merkez (90°)"),
            (2000, "135°"),
            (2500, "Maksimum (180°)")
        ]
        
        for micros, description in test_positions:
            print(f"📍 Test: {description} - {micros}μs")
            self.send_servo_command(micros, micros, use_micros=True)
            time.sleep(1.5)
        
        print("🎯 Merkeze dönülüyor...")
        self.center_camera()
        print("✅ Kalibrasyon tamamlandı!")
    
    def fine_tune_position(self):
        """İnteraktif hassas ayarlama modu"""
        print("\n" + "="*60)
        print("🎛️  HASSAS AYARLAMA MODU")
        print("="*60)
        print("Komutlar:")
        print("  pan <derece>   - Pan pozisyonunu ayarla (örn: pan 84.5)")
        print("  tilt <derece>  - Tilt pozisyonunu ayarla (örn: tilt 150.2)")
        print("  pus <mikros>   - Pan'ı mikrosaniye ile ayarla")
        print("  tus <mikros>   - Tilt'i mikrosaniye ile ayarla")
        print("  smooth <pan> <tilt> - Yumuşak hareket")
        print("  status         - Mevcut pozisyon")
        print("  exit           - Çıkış")
        print("="*60)
        
        while True:
            try:
                cmd = input("Komut> ").strip().lower().split()
                
                if not cmd:
                    continue
                    
                if cmd[0] == 'exit':
                    break
                    
                elif cmd[0] == 'pan' and len(cmd) == 2:
                    pan_deg = float(cmd[1])
                    self.precise_position(pan_deg, self.current_tilt)
                    
                elif cmd[0] == 'tilt' and len(cmd) == 2:
                    tilt_deg = float(cmd[1])
                    self.precise_position(self.current_pan, tilt_deg)
                    
                elif cmd[0] == 'pus' and len(cmd) == 2:
                    pan_us = int(cmd[1])
                    self.send_servo_command(pan_us, self.current_tilt_us, use_micros=True)
                    
                elif cmd[0] == 'tus' and len(cmd) == 2:
                    tilt_us = int(cmd[1])
                    self.send_servo_command(self.current_pan_us, tilt_us, use_micros=True)
                    
                elif cmd[0] == 'smooth' and len(cmd) == 3:
                    target_pan = float(cmd[1])
                    target_tilt = float(cmd[2])
                    self.smooth_move_to(target_pan, target_tilt)
                    
                elif cmd[0] == 'status':
                    print(f"📊 Pozisyon: Pan={self.current_pan:.2f}° ({self.current_pan_us}μs), "
                          f"Tilt={self.current_tilt:.2f}° ({self.current_tilt_us}μs)")
                    
                else:
                    print("❌ Geçersiz komut!")
                    
            except ValueError as e:
                print(f"❌ Hata: {e}")
            except KeyboardInterrupt:
                break
        
        print("Hassas ayarlama modundan çıkıldı.")
    
    def detect_and_track_bullseye(self, frame):
        """YOLO ile bullseye tanıma ve gelişmiş kilitleme sistemi"""
        results = model(frame, conf=self.confidence_threshold, verbose=False)
        
        current_time = time.time()
        bullseye_detections = []
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    class_id = int(box.cls[0])
                    class_name = model.names[class_id]
                    confidence = float(box.conf[0])
                    
                    if class_name.lower() == self.bullseye_class_name.lower() or "bullseye" in class_name.lower():
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        width = x2 - x1
                        height = y2 - y1
                        center_x = int(x1 + width / 2)
                        center_y = int(y1 + height / 2)
                        
                        bullseye_detections.append({
                            'bbox': (int(x1), int(y1), int(width), int(height)),
                            'center': (center_x, center_y),
                            'confidence': confidence,
                            'size': max(width, height)
                        })
        
        if len(bullseye_detections) > 0:
            # Hedef bulundu
            self.last_bullseye_detection_time = current_time
            self.lost_target_recovery = False
            self.target_lost_time = None  # Hedef bulundu, kayıp zamanını sıfırla
            
            largest_bullseye = max(bullseye_detections, key=lambda d: d['size'])
            
            x, y, w, h = largest_bullseye['bbox']
            center_x, center_y = largest_bullseye['center']
            confidence = largest_bullseye['confidence']
            size = largest_bullseye['size']
            
            self.target_box = (x, y, w, h)
            self.last_known_target_center = (center_x, center_y)  # Son bilinen merkezi güncelle
            
            # Zoom kontrolü - DÜZELTILMIŞ MANTIK
            deadzone_to_target_ratio = (self.dead_zone_size * 2) / size if size > 0 else float('inf')  # Deadzone'un target'a oranı
            
            # Zoom IN kontrolü - deadbox targetbox'un 0.7'sinden büyükse (hedef küçük)
            if deadzone_to_target_ratio > self.zoom_in_threshold:
                if self.zoom_level < self.zoom_max:
                    self.zoom_level = min(self.zoom_max, self.zoom_level + 0.05)
                    print(f"🔍 Zoom IN: {self.zoom_level:.1f}x (Hedef küçük, DZ/Target oranı: {deadzone_to_target_ratio:.2f})")
            
            # Zoom OUT kontrolü - deadbox targetbox'un 0.35'inden küçükse (hedef çok büyük)
            elif deadzone_to_target_ratio < self.zoom_out_threshold:
                if self.zoom_level > self.zoom_min:
                    self.zoom_level = max(self.zoom_min, self.zoom_level - 0.05)
                    print(f"🔍 Zoom OUT: {self.zoom_level:.1f}x (Hedef büyük, DZ/Target oranı: {deadzone_to_target_ratio:.2f})")
            
            # Kilitleme kontrolü
            if not self.target_locked:
                # Hedef boyutu ve deadzone içinde olma kontrolü
                if deadzone_to_target_ratio <= self.zoom_in_threshold and self.is_dead_zone_inside_target(self.target_box):
                    self.target_locked = True
                    print("🎯 HEDEF KİLİTLENDİ!")
                else:
                    # Hedefi merkeze getir
                    if (current_time - self.last_bullseye_move_time) > self.bullseye_move_interval:
                        self.track_to_target_center(center_x, center_y)
                        self.last_bullseye_move_time = current_time
            else:
                # Kilit kontrolü
                if not self.is_dead_zone_inside_target(self.target_box):
                    self.target_locked = False
                    print("⚠️ Kilit kayboldu, yeniden hedefleniyor...")
                else:
                    # Kilitliyken de hedefi merkeze getirmeye devam et
                    frame_center_x = self.frame_width // 2
                    frame_center_y = self.frame_height // 2
                    dist_x = abs(center_x - frame_center_x)
                    dist_y = abs(center_y - frame_center_y)
                    
                    if dist_x > 5 or dist_y > 5:
                        if (current_time - self.last_bullseye_move_time) > self.bullseye_move_interval * 2:
                            self.track_to_target_center(center_x, center_y)
                            self.last_bullseye_move_time = current_time
            
            # Sadece kutu çizimi (yazılar arayüzde)
            if self.target_locked:
                color = (0, 255, 0)
                thickness = 3
            else:
                color = (0, 255, 255)
                thickness = 2
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, thickness)
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
            
        else:
            # Hedef kayıp
            if self.target_lost_time is None:
                self.target_lost_time = current_time  # İlk kayıp anını kaydet
            
            time_since_lost = current_time - self.target_lost_time if self.target_lost_time else 0
            
            # 3 saniye boyunca son bilinen yöne bakmaya devam et
            if time_since_lost < self.continue_tracking_duration and self.last_known_target_center:
                if (current_time - self.last_bullseye_move_time) > self.bullseye_move_interval:
                    center_x, center_y = self.last_known_target_center
                    self.track_to_target_center(center_x, center_y)
                    self.last_bullseye_move_time = current_time
                    print(f"📍 Son bilinen yöne bakılıyor... ({3 - time_since_lost:.1f}s kaldı)")
            else:
                # 3 saniye geçti veya daha uzun süre kayıp
                self.target_locked = False
                self.target_box = None
                
                time_since_last_bullseye = current_time - self.last_bullseye_detection_time
                
                if time_since_last_bullseye > self.no_bullseye_timeout and not self.lost_target_recovery:
                    print(f"⚠️ {self.no_bullseye_timeout} saniyedir bullseye bulunamadı! Merkeze dönülüyor...")
                    self.center_camera()
                    self.zoom_level = 1.0
                    self.lost_target_recovery = True
        
        # Dead zone çizimi - Dairesel
        frame_center_x = self.frame_width // 2
        frame_center_y = self.frame_height // 2
        dz_color = (0, 255, 0) if self.target_locked else (0, 255, 255)
        cv2.circle(frame, 
                  (frame_center_x, frame_center_y),
                  self.dead_zone_size,
                  dz_color, 2 if self.target_locked else 1)
        
        return frame
    
    def draw_interface(self, frame):
        """Geliştirilmiş arayüz çizimi - TÜM YAZILAR SOL ÜSTTE"""
        center_x = self.frame_width // 2
        center_y = self.frame_height // 2
        cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
        cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)
        
        # Sol üst köşede bilgi paneli
        y_offset = 30
        line_height = 20
        
        # Mod bilgisi
        mode_text = "BULLSEYE TAKİP" if self.bullseye_tracking else "MANUEL KONTROL"
        mode_color = (0, 255, 0) if self.bullseye_tracking else (255, 255, 255)
        cv2.putText(frame, mode_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
        y_offset += line_height + 5
        
        # Pozisyon bilgisi
        cv2.putText(frame, f"Pan: {self.current_pan:.2f}° ({self.current_pan_us}μs)", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += line_height
        
        cv2.putText(frame, f"Tilt: {self.current_tilt:.2f}° ({self.current_tilt_us}μs)", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += line_height
        
        # Kontrol bilgileri
        cv2.putText(frame, f"Adım: {self.step_size:.2f}°" if not self.use_micros_mode else f"Adım: {self.micros_step}μs", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        y_offset += line_height
        
        cv2.putText(frame, f"Zoom: {self.zoom_level:.1f}x", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        y_offset += line_height
        
        # Hedef bilgileri (bullseye modundaysa)
        if self.bullseye_tracking:
            lock_text = "🔒 KİLİTLİ" if self.target_locked else "🔓 KİLİTSİZ"
            lock_color = (0, 255, 0) if self.target_locked else (0, 0, 255)
            cv2.putText(frame, lock_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, lock_color, 2)
            y_offset += line_height
            
            # Hedef bilgisi varsa
            if self.target_box:
                x, y, w, h = self.target_box
                cv2.putText(frame, f"Hedef Boyut: {w:.0f}x{h:.0f}px", 
                           (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                y_offset += line_height
                
                # Oran bilgisi - DÜZELTILMIŞ
                deadzone_to_target_ratio = (self.dead_zone_size * 2) / max(w, h) if max(w, h) > 0 else float('inf')
                ratio_color = (0, 255, 0) if deadzone_to_target_ratio <= self.zoom_in_threshold else (0, 0, 255)
                cv2.putText(frame, f"DZ/Target Oranı: {deadzone_to_target_ratio:.2f} (Hedef: ≤0.7)", 
                           (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ratio_color, 1)
                y_offset += line_height
            
            # Hedef kayıpsa süre bilgisi
            if self.target_lost_time:
                time_since_lost = time.time() - self.target_lost_time
                if time_since_lost < self.continue_tracking_duration:
                    remaining = self.continue_tracking_duration - time_since_lost
                    cv2.putText(frame, f"Son yöne bakılıyor: {remaining:.1f}s", 
                               (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    y_offset += line_height
        
        # YOLO güven seviyesi
        cv2.putText(frame, f"YOLO Güven: {self.confidence_threshold:.1f}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        y_offset += line_height
        
        # Dead Zone boyutu
        cv2.putText(frame, f"Dead Zone: {self.dead_zone_size}px (sabit)", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        y_offset += line_height
        
        # Kontrol modu
        mode = "MİKROSANİYE" if self.use_micros_mode else "DERECE"
        cv2.putText(frame, f"Kontrol: {mode}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # Kontroller (sağ alt köşede)
        y_start = frame.shape[0] - 260
        cv2.putText(frame, "KONTROLLER:", (10, y_start), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, "W/A/S/D: Manuel hareket", (10, y_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(frame, "[ / ]: Adım boyutunu azalt/arttır", (10, y_start + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        cv2.putText(frame, "SPACE: Bullseye takip modu", (10, y_start + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(frame, "+ / -: Manuel zoom", (10, y_start + 65), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        cv2.putText(frame, "C: Merkeze dön", (10, y_start + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(frame, "T/G: YOLO güven ayarı", (10, y_start + 95), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
        cv2.putText(frame, "R: Zoom reset", (10, y_start + 110), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        cv2.putText(frame, "M: Mikrosaniye/Derece modu", (10, y_start + 125), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        cv2.putText(frame, "F: Hassas ayarlama modu", (10, y_start + 140), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        cv2.putText(frame, "K: Kalibrasyon", (10, y_start + 155), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
        cv2.putText(frame, "P: Pozisyon bilgisi", (10, y_start + 170), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        cv2.putText(frame, "Q: Çıkış", (10, y_start + 185), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        
        return frame
    
    def center_camera(self):
        """Kamerayı merkeze getir - YENİ MERKEZ DEĞERLERİ"""
        print("Kamera merkeze getiriliyor...")
        self.current_pan = 114.0  # Yeni merkez pan
        self.current_tilt = 14.0  # Yeni merkez tilt
        self.send_servo_command(self.current_pan, self.current_tilt)
        self.target_locked = False
        self.target_box = None
        self.last_bullseye_detection_time = time.time()
        self.lost_target_recovery = False
        self.last_known_target_center = None
        self.target_lost_time = None
    
    def run(self):
        """Ana döngü - MG995 hassas kontrol versiyonu"""
        if not self.initialize_camera():
            return
        
        cv2.namedWindow('MG995 Precision Bullseye Tracker')
        
        print("=" * 70)
        print("🎯 MG995 PRECISION BULLSEYE TRACKER")
        print("=" * 70)
        print(f"ESP32 IP: {self.esp32_ip}")
        print(f"Servo Merkezi: Pan={114}°, Tilt={14}°")
        print(f"Servo Aralığı: {self.SERVO_MIN_US}-{self.SERVO_MAX_US}μs (MG995)")
        print("\n📌 HASSAS KONTROL ÖZELLİKLERİ:")
        print("- 0.1° hassasiyetle pozisyonlama")
        print("- Mikrosaniye tabanlı kontrol (1μs hassasiyet)")
        print("- Hedef kaybolduğunda 3 saniye son yöne bakma")
        print("- Otomatik zoom in/out kontrolü")
        print("\n🎮 KONTROLLER:")
        print("W/A/S/D: Yukarı/Sol/Aşağı/Sağ hareket")
        print("[ / ]: Adım boyutunu azalt/arttır")
        print("M: Mikrosaniye/Derece modu geçişi")
        print("F: İnteraktif hassas ayarlama modu")
        print("K: Servo kalibrasyon testi")
        print("SPACE: Bullseye takip modunu aç/kapat")
        print("+ / -: Zoom in/out")
        print("C: Merkeze dön")
        print("R: Zoom reset")
        print("T/G: YOLO güven seviyesi ayarı")
        print("P: Pozisyon bilgisini göster")
        print("1-9: Hızlı pozisyonlama")
        print("Q: Çıkış")
        print("=" * 70)
        
        self.running = True
        
        while self.running:
            ret, frame = self.camera.read()
            if not ret:
                print("Kamera görüntüsü alınamıyor!")
                break
            
            frame = self.apply_zoom(frame)
            
            if self.bullseye_tracking:
                frame = self.detect_and_track_bullseye(frame)
            
            frame = self.draw_interface(frame)
            
            cv2.imshow('MG995 Precision Bullseye Tracker', frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                self.running = False
            
            elif not self.bullseye_tracking:
                if key == ord('w'):
                    self.manual_move('w')
                elif key == ord('s'):
                    self.manual_move('s')
                elif key == ord('a'):
                    self.manual_move('a')
                elif key == ord('d'):
                    self.manual_move('d')
            
            if key == ord('['):
                self.adjust_step_size(False)
            elif key == ord(']'):
                self.adjust_step_size(True)
            
            elif key == ord('m'):
                self.toggle_micros_mode()
            
            elif key == ord('f'):
                print("\n🎛️  Hassas ayarlama moduna geçiliyor...")
                self.fine_tune_position()
                print("Ana moda dönüldü.\n")
            
            elif key == ord('k'):
                self.calibrate_servo_range()
            
            elif key == ord('p'):
                print(f"\n📊 MEVCUT POZİSYON:")
                print(f"  Pan:  {self.current_pan:.2f}° ({self.current_pan_us}μs)")
                print(f"  Tilt: {self.current_tilt:.2f}° ({self.current_tilt_us}μs)")
                print(f"  Adım: {self.step_size:.2f}° veya {self.micros_step}μs")
                print(f"  Mod:  {'Mikrosaniye' if self.use_micros_mode else 'Derece'}\n")
            
            elif key == ord(' '):
                self.bullseye_tracking = not self.bullseye_tracking
                if self.bullseye_tracking:
                    print("🎯 BULLSEYE TAKİP MODU AKTİF (Hassas takip)")
                    self.target_locked = False
                    self.target_box = None
                else:
                    print("🎮 MANUEL KONTROL MODU AKTİF (Hassas kontrol)")
                self.last_bullseye_detection_time = time.time()
                self.lost_target_recovery = False
                self.last_known_target_center = None
                self.target_lost_time = None
            
            elif key == ord('+') or key == ord('='):
                self.zoom_level = min(self.zoom_max, self.zoom_level + self.zoom_step)
                print(f"Zoom: {self.zoom_level:.1f}x")
            elif key == ord('-'):
                self.zoom_level = max(self.zoom_min, self.zoom_level - self.zoom_step)
                print(f"Zoom: {self.zoom_level:.1f}x")
            elif key == ord('r'):
                self.zoom_level = 1.0
                print("Zoom reset edildi")
            
            elif key == ord('c'):
                self.center_camera()
            
            elif key == ord('t'):
                self.confidence_threshold = min(0.9, self.confidence_threshold + 0.1)
                print(f"YOLO güven seviyesi: {self.confidence_threshold:.1f}")
            elif key == ord('g'):
                self.confidence_threshold = max(0.1, self.confidence_threshold - 0.1)
                print(f"YOLO güven seviyesi: {self.confidence_threshold:.1f}")
            
        
        self.cleanup()
    
    def cleanup(self):
        """Temizleme işlemleri"""
        print("Temizlik yapılıyor...")
        if self.camera:
            self.camera.release()
        cv2.destroyAllWindows()
        print("Program sonlandırıldı.")

if __name__ == "__main__":
    # ESP32'nizin IP adresini buraya yazın
    esp32_ip = "192.168.43.185"
    
    controller = PanTiltController(esp32_ip)
    
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\nProgram sonlandırılıyor...")
        controller.cleanup()
