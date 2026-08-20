import cv2
import numpy as np

class QRDetector:
    def __init__(self):
        # Necháme spadnout, pokud detektor není dostupný (fail-fast)
        self.wechat_detector = cv2.wechat_qrcode_WeChatQRCode()

    def _try_decode(self, image):
        res, points = self.wechat_detector.detectAndDecode(image)
        if res and len(res) > 0 and res[0]:
            return [data for data in res if data], points
        return None, points

    def detect(self, img):
        if img is None:
            return []
            
        # 1. Původní obraz (nejčastější scénář)
        result, points = self._try_decode(img)
        if result: return result
        
        # Pokud nenašel ani oblast, kde by QR kód mohl být, ukonči to ihned.
        if not points or len(points) == 0:
            return []
            
        # Získání ořezu oblasti s potenciálním QR kódem (pro zrychlení zbývajících kroků)
        pts = np.array(points[0], dtype=np.int32)
        x, y, w, h = cv2.boundingRect(pts)
        
        # Přidáme okraj 20px
        pad = 20
        h_img, w_img = img.shape[:2]
        x1 = max(0, x - pad)
        y1 = max(0, y - pad)
        x2 = min(w_img, x + w + pad)
        y2 = min(h_img, y + h + pad)
        
        crop = img[y1:y2, x1:x2]
        if crop.size == 0:
            return []
            
        # 2. Převod do šedi (pouze na ořezu)
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        result, _ = self._try_decode(gray)
        if result: return result
        
        # 3. Gaussian Blur + CLAHE
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        clahe_img = clahe.apply(blur)
        result, _ = self._try_decode(clahe_img)
        if result: return result
        
        # 4. Rotace o 45 stupňů (řeší kosočtvercové a natočené kódy)
        (h_crop, w_crop) = gray.shape[:2]
        center = (w_crop // 2, h_crop // 2)
        M = cv2.getRotationMatrix2D(center, 45, 1.0)
        rotated_45 = cv2.warpAffine(gray, M, (w_crop, h_crop))
        result, _ = self._try_decode(rotated_45)
        if result: return result
        
        # 5. Adaptivní prahování
        thresh = cv2.adaptiveThreshold(clahe_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                       cv2.THRESH_BINARY, 21, 5)
        result, _ = self._try_decode(thresh)
        if result: return result
        
        # 6. Morfologie (Closing)
        kernel = np.ones((3,3), np.uint8)
        morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        result, _ = self._try_decode(morph)
        if result: return result
        
        return []
