import cv2
import os
import glob

# 1. Configuración de directorios y dimensiones
ruta_origen = "C:\\Users\\juana\\OneDrive - INTEC\\Documentos\\PIGEON_INVADERS\\Detection Model\\Paloma\\"      # Carpeta con tus fotos originales de Telegram
ruta_destino = "C:\\Users\\juana\\OneDrive - INTEC\\Documentos\\PIGEON_INVADERS\\Detection Model\\Paloma_Crop\\"     # Carpeta donde se guardarán los recortes
ancho_roi = 280
alto_roi = 280

# 2. Coordenadas de inicio (redondeadas a enteros)
x_inicio = 106
y_inicio = 79

# Crear la carpeta de destino si no existe
os.makedirs(ruta_destino, exist_ok=True)

# Obtener todas las imágenes .jpg de la carpeta origen
imagenes = glob.glob(os.path.join(ruta_origen, "*.jpg"))

print(f"Iniciando el recorte de {len(imagenes)} imágenes...")

for ruta_img in imagenes:
    # Leer la imagen original
    img = cv2.imread(ruta_img)
    
    if img is not None:
        # 3. Aplicar el recorte: OpenCV usa el formato imagen[y_inicio:y_fin, x_inicio:x_fin]
        recorte = img[y_inicio : y_inicio + alto_roi, x_inicio : x_inicio + ancho_roi]
        
        # Extraer solo el nombre del archivo y guardar el recorte
        nombre_archivo = os.path.basename(ruta_img)
        ruta_guardado = os.path.join(ruta_destino, nombre_archivo)
        cv2.imwrite(ruta_guardado, recorte)
    else:
        print(f"Error al leer la imagen: {ruta_img}")

print("¡Procesamiento masivo completado con éxito!")