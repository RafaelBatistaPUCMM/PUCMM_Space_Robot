import cv2 as cv
import numpy as np

def object_segmentation(dist_img, rgb_img, max_range, cloud, display=False):
    #Distance Filtering
    D_mask = dist_img[:,:,0]
    D_th_max = int(255*4/max_range) #4 meter threashold
    D_th_min = int(255*2/max_range) #2 meter threashold

    logic = np.bitwise_and(D_mask <= D_th_max, D_mask >= D_th_min)
    mask_D = np.where(logic, 255, 0).astype('uint8')
    rgb_masked_D = cv.bitwise_and(rgb_img, rgb_img, mask=mask_D)

    #Display
    if display:
        cv.imshow("D mask",mask_D)
        cv.imshow("RGB masked", rgb_masked_D)

    #Saturation Filtering
    hsv_img = cv.cvtColor(rgb_masked_D,cv.COLOR_BGR2HSV)
    sat = hsv_img[:,:,1]

    #sat = np.where(sat > 128, 255, 0).astype('uint8')
    ret, sat1 = cv.threshold(sat,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)

    #Display
    if display:
        cv.imshow("Sat Img 1", sat1)

    SE = np.ones((3,3),np.uint8)
    sat2 = cv.erode(sat1, SE)#Erosion
    sat2 = cv.medianBlur(sat2, 3)#Filtro de mediana
    sat2 = cv.dilate(sat2, SE)#Dilatacion
    
    #Display
    if display:
        cv.imshow("Sat Img 2", sat2)

    SE = np.ones((5,5),np.uint8)
    sat3 = cv.erode(sat2, SE)#Erosion
    sat3 = cv.GaussianBlur(sat2,(5,5), cv.BORDER_DEFAULT)#Filtro Gaussiano
    ret, sat3 = cv.threshold(sat3,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)#Thresholding

    #Display
    if display:
        cv.imshow("Sat Img 3", sat3)


    contours, hierarchies = cv.findContours(sat3, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    centroids = []

    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > 50:
            mask = np.zeros(sat3.shape, dtype=np.uint8)
            mask = cv.drawContours(mask,[cnt],0,255,-1)

            xyz = []
            for i in range(mask.shape[0]):
                for j in range(mask.shape[1]):
                    if mask[i][j] == 255:
                        x_px = cloud[i][j][0]
                        y_px = cloud[i][j][1]
                        z_px = cloud[i][j][2]
                        xyz_px = np.array([x_px, y_px, z_px])
                        if not np.any(np.isnan(xyz_px)):
                            xyz.append(xyz_px)
            xyz = np.array(xyz)
            cent = xyz.mean(axis=0)
            centroids.append(cent)

    return np.array(centroids)

def test_obj_seg():
    #Test Values Start
    #f = 'Log_121122_1729/ZonaZona2_/Zona2_'
    f = 'Log_121122_1802/Zona1/Zona1_'
    cloud = np.load(f+'CloudXYZ.npy')
    dist = np.load(f+'DistData.npy')
    max_range = np.load(f+'MaxRange.npy')
    dist_img = cv.imread(f+'Dist.jpg')
    rgb_img = cv.imread(f+'RGB.jpg')
    rgb_img = cv.cvtColor(rgb_img, cv.COLOR_RGB2BGR)
    #Test Values End

    c = ObjectSegmentation(dist_img, rgb_img, max_range, cloud, True)

    print(len(c))
    for cx in c:
        print(cx)

test_obj_seg()
