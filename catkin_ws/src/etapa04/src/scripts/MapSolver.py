from scipy.io import loadmat

import numpy as np
import cv2 as cv
import time

robot_zone = 0.47 #metros

def obtain_data(filename):
    return loadmat(filename)

def CompareColor(data, r, g, b):
    if len(data.shape) == 3:
        return np.any(np.bitwise_and(data[:,:,0] == b, data[:,:,1] == g, data[:,:,2] == r))
    elif len(data.shape) == 1:
        return np.any(data[0] == b and data[1] == g and data[2] == r)
    
def vecindades(mapa):
    ### Testing
    #t1 = time.time()
    ### Testing
    
    resolution = mapa['resolution']
    width = int(mapa['width'])
    height = int(mapa['height'])
    data = mapa['map']

    image = np.where(data == 0, 255, data)
    image = np.where(image == 100, 0, image)
    image = np.where(image == -1, 100, image)
    image = np.dstack((image,image,image))
    image = image.astype(np.uint8)

    f_min = height
    f_max = 1
    c_min = width
    c_max = 1

    get_in_condition = False
    for f in range(1,height-1):
        cond_out = np.all(data[f,:] == -1)
        if not cond_out:
            for c in range(1,width-1):
                if not np.all(data[:,c] == -1):
                    vecinos = data[f-1:f+2, c-1:c+2]
                    cond_1 = np.any(vecinos == -1)
                    cond_2 = np.any(vecinos == 100)
                    cond_3 = np.any(vecinos == 0)
                    
                    if data[f,c] == 100 or data[f,c] == 0:
                        get_in_condition = True
                        
                    if cond_1 and cond_2 and cond_3:
                        image[f-1:f+2, c-1:c+2, 0] = 0
                        image[f-1:f+2, c-1:c+2, 1] = 0
                        image[f-1:f+2, c-1:c+2, 2] = 255
                        
                    elif cond_1 and (not cond_2) and cond_3:
                        image[f-1:f+2, c-1:c+2, 0] = 0
                        image[f-1:f+2, c-1:c+2, 1] = 255
                        image[f-1:f+2, c-1:c+2, 2] = 0


                    if cond_2:
                        if f > f_max:
                            f_max = f
                        
                        elif f < f_min:
                            f_min = f
                        
                        if c > c_max:
                            c_max = c
                        
                        elif c < c_min:
                            c_min = c
        if get_in_condition and cond_out:
            break
        
    for f in range(f_min,f_max+1):
        for c in range(c_min,c_max+1):
            vecinos = image[f-1:f+2, c-1:c+2, :]
            
            if CompareColor(vecinos[1,1,:],0,255,0) and (not CompareColor(vecinos,100,100,100)):
                image[f,c,0] = 255
                image[f,c,1] = 255
                image[f,c,2] = 255

    
    ###Testing
    #print("Elapsed", time.time()-t1)
    #cv.imshow("Test", image[f_min:f_max+1,c_min:c_max+1,:])
    ###Testing
    return image, [f_min, f_max, c_min, c_max]

def interface_lines(image, limits):
    green = np.bitwise_and(image[:,:,0] == 0,image[:,:,1] == 255,image[:,:,2] == 0)
    
    ###For verification
    #cv.imshow("Original",image)
    #cv.imshow("Then",green.astype(np.uint8) * 255)
    ###For verification 
    
    f_min = limits[0]
    f_max = limits[1]
    c_min = limits[2]
    c_max = limits[3]

    row_pos = np.array([[-1, -1, -1],[0,0,0],[1,1,1]])
    col_pos = row_pos.T

    inicios = []
    finales = []
    for f in range(f_min,f_max+1):
        for c in range(c_min,c_max+1):
            vecinos = green[f-1:f+2, c-1:c+2].copy()
            vecinos[0,0] = False
            vecinos[2,2] = False
            vecinos[0,2] = False
            vecinos[2,0] = False
            
            if vecinos[1,1]:
                #Inicio de una linea
                inicio = [f,c]
                idf = f
                idc = c
                stuck = 0
                while True:
                    vecinos[1,1] = False
            
                    green[idf,idc] = False
                    
                    rp = row_pos[vecinos]
                    cp = col_pos[vecinos]

                    if rp.shape[0] == 0 or cp.shape[0] == 0:
                        break
                    else:
                        idf += rp[0]
                        idc += cp[0]
                        final = [idf, idc]
                    
                    vecinos = green[idf-1:idf+2, idc-1:idc+2].copy()
                    vecinos[0,0] = False
                    vecinos[2,2] = False
                    vecinos[0,2] = False
                    vecinos[2,0] = False

                inicios.append(inicio)
                finales.append(final)
    
    inicios = np.array(inicios)
    finales = np.array(finales)

    ###For verification
    #cv.imshow("Now",green.astype(np.uint8) * 255)
    ###For verification 
    
    return inicios, finales
                        
def distance_discard(mapa, inicios, finales):
    resolution = mapa['resolution']

    ###For Testing
    #data = mapa['map']

    #image = np.where(data == 0, 255, data)
    #image = np.where(image == 100, 0, image)
    #image = np.where(image == -1, 100, image)
    #image = np.dstack((image,image,image))
    #image = image.astype(np.uint8)
    #img = np.zeros(image.shape, np.uint8)
    ###For Testing
    
    distances = np.array([np.sqrt(np.sum(((finales - inicios)*resolution) ** 2, 1))]).T

    boolean = distances > robot_zone
    inicios_cut = []
    finales_cut = []
    distances_cut = []
    for i in range(0,len(boolean[:,0])):
        if boolean[i,0]:
            inicios_cut.append(inicios[i,:])
            finales_cut.append(finales[i,:])
            distances_cut.append(distances[i,:])

            ###For Testing
            #p0 = (inicios[i,1], inicios[i,0])
            #p1 = (finales[i,1],  finales[i,0])
            #img = cv.line(img, p0, p1,(0, 255, 255), 2)
            ###For Testing

    inicios_cut = np.array(inicios_cut)
    finales_cut = np.array(finales_cut)
    distances_cut = np.array(distances_cut)

    
    ###For Testing
    #img = np.where(img == 0, image, img)
    #cv.imshow("Test2", img)
    ###For Testing
    return distances_cut, inicios_cut, finales_cut

def view_pointer(mapa, inicios, finales, distances, limits):
    #print(finales, inicios)
    if 0 in inicios.shape:
    	return inicios, inicios
   	
    resolution = mapa['resolution']
    width = int(mapa['width'])
    height = int(mapa['height'])
    data = mapa['map']
    position = mapa['position']

    f_min = limits[0]
    f_max = limits[1]
    c_min = limits[2]
    c_max = limits[3]
    
    image = np.where(data == 0, 255, data)
    image = np.where(image == 100, 0, image)
    image = np.where(image == -1, 100, image)
    image = image.astype(np.uint8)

    ext_inicios = inicios.copy()
    ext_finales = finales.copy()

    mid = (inicios + finales) / 2
    mid = mid.astype(np.int64)

    difY = finales[:,0] - inicios[:,0]
    difX = finales[:,1] - inicios[:,1]
    difX[difX == 0] = 1e-10
    slope = difY / difX
    slope[slope == 0] = 1e-10
    inv_slope = -1 / slope;

    ang = np.arctan(inv_slope)
    dx1 = np.cos(ang)
    dy1 = np.sin(ang)

    ang += np.pi
    dx2 = np.cos(ang)
    dy2 = np.sin(ang)

    f_mat = np.ones(data.shape,dtype=np.int64) * np.array([np.arange(height)]).T
    c_mat = np.ones(data.shape,dtype=np.int64) * np.array(np.arange(width))

    spot_size = int(np.ceil(1.5*robot_zone/resolution))
    spot_center = int(np.ceil(spot_size/2) - 1)
    spot_radius = int(np.ceil(robot_zone/(resolution*2)))
    robot_spot = cv.circle(np.zeros((spot_size,spot_size),dtype=np.uint8), (spot_center, spot_center), spot_radius, 255, cv.FILLED)

    robot_offset = np.ceil((len(robot_spot[:,0]) - np.ceil(robot_zone/resolution + 1))/2)
    robot_offset = int(robot_offset[0][0])

    spot_off = np.around(((np.array(robot_spot.shape) - 1)/2))
    spot_off = spot_off.astype(np.int64)

    centroids = []
    
    for i in range(len(slope)):
        #Vecindad 1
        offset = np.array([dy1[i], dx1[i]]) * 10
        endpoint = np.around(mid[i,:] + offset).astype(np.int64)
        
        if endpoint[0] > mid[i,0]:
            startf = mid[i,0]
            endf = endpoint[0]
        else:
            endf = mid[i,0]
            startf = endpoint[0]

        if endpoint[1] > mid[i,1]:
            startc = mid[i,1]
            endc = endpoint[1]
        else:
            endc = mid[i,1]
            startc = endpoint[1]

        vecinos1 = image[startf:endf, startc:endc]
        
        #Vecindad 2
        offset = np.array([dy2[i], dx2[i]]) * 10
        endpoint = np.around(mid[i,:] + offset).astype(np.int64)

        if endpoint[0] > mid[i,0]:
            startf = mid[i,0]
            endf = endpoint[0]
        else:
            endf = mid[i,0]
            startf = endpoint[0]

        if endpoint[1] > mid[i,1]:
            startc = mid[i,1]
            endc = endpoint[1]
        else:
            endc = mid[i,1]
            startc = endpoint[1]

        vecinos2 = image[startf:endf, startc:endc]

        if np.sum(vecinos1 == 255) > np.sum(vecinos2 == 255):
            dx = dx1[i]
            dy = dy1[i]
        else:
            dx = dx2[i]
            dy = dy2[i]

        
        #Proyeccion
        f = mid[i,0]
        c = mid[i,1]
        offx = dx
        offy = dy
        
        while f < f_max and c < c_max and f > f_min and c > c_min and (not (image[f,c] == 0)):
            f = int(round(mid[i,0] + offy))
            c = int(round(mid[i,1] + offx))
            offx += dx
            offy += dy

        if f < f_min:
            f = f_min
        elif f > f_max:
            f = f_max

        if c < c_min:
            c = c_min
        elif c > c_max:
            c = c_min

        black_point = [f,c]

        ang1 = np.arctan(slope[i])
        ang2 = np.pi + ang1

        shift1 = distances[i,0] * np.array([np.sin(ang1), np.cos(ang1)]) / (2*resolution)
        shift2 = distances[i,0] * np.array([np.sin(ang2), np.cos(ang2)]) / (2*resolution)

        shift1 = np.around(shift1)[0]
        shift2 = np.around(shift2)[0]

        
        if shift2[0] > shift1[0]:
            bubble = shift1
            shift1 = shift2
            shift2 = bubble

        ext_inicios[i,:] = black_point + shift1
        ext_finales[i,:] = black_point + shift2

        blank = image.copy()
        blank[blank != 255] = 0

        mask = np.zeros((height,width),dtype=np.uint8)
        pts = [(inicios[i,1], inicios[i,0]), (finales[i,1], finales[i,0]), (ext_inicios[i,1], ext_inicios[i,0]), (ext_finales[i,1], ext_finales[i,0])]
        mask = cv.fillPoly(mask, np.array([pts]), 255)

        area = np.bitwise_and(mask, blank)
        
        fs = f_mat[area == 255]
        cs = c_mat[area == 255]

        if len(fs) > 0:
            fs_min = np.min(fs)
            fs_max = np.max(fs)
            cs_min = np.min(cs)
            cs_max = np.max(cs)
        
        working_section = area[fs_min-robot_offset:fs_max+robot_offset, cs_min-robot_offset:cs_max+robot_offset]

        fmax, cmax = np.array(working_section.shape) - np.array(robot_spot.shape)
        robot_f, robot_c = np.array(robot_spot.shape)
        section_f, section_c = np.array(working_section.shape)

        marks = np.zeros(working_section.shape, np.uint8)
        for f in range(fmax):
            for c in range(cmax):
                zero1 = np.zeros((robot_f, c),np.uint8)
                zero2 = np.zeros((robot_f, section_c - c - robot_c),np.uint8)
                zero3 = np.zeros((f, section_c),np.uint8)
                zero4 = np.zeros((section_f - f - robot_f, section_c),np.uint8)

                zero_conds = [0 in zero1.shape, 0 in zero2.shape,0 in zero3.shape, 0 in zero4.shape]
                if not zero_conds[0] and not zero_conds[1]:
                    pad1 = np.concatenate((zero1, robot_spot, zero2), axis=1)
                elif zero_conds[0]:
                    pad1 = np.concatenate((robot_spot, zero2), axis=1)
                elif zero_conds[1]:
                    pad1 = np.concatenate((zero1, robot_spot), axis=1)
                else:
                    pad1 = robot_spot

                if not zero_conds[2] and not zero_conds[3]:
                    pad2 = np.concatenate((zero3, pad1, zero4), axis=0)
                elif zero_conds[2]:
                    pad2 = np.concatenate((pad1, zero4), axis=0)
                elif zero_conds[3]:
                    pad2 = np.concatenate((zero3, pad1), axis=0)
                else:
                    pad2 = pad1

                or_op = np.bitwise_or(pad2, working_section)
                if np.all(or_op == working_section):
                    marks[f,c] = 255

        if np.any(marks == 255):
            M = cv.moments(marks)
            cX = int(round(M["m10"] / M["m00"]) + spot_off[1])
            cY = int(round(M["m01"] / M["m00"]) + spot_off[0])

            cF = cY + fs_min - robot_offset
            cC = cX + cs_min - robot_offset
            
            centroids.append([cF,cC])
            
    ###For Testing
    #image = np.dstack((image,image,image))
    #mask = np.zeros(image.shape, np.uint8)
    ###For Testing
    
    view_points = []
    for cent in centroids:
        cX = cent[1]*resolution[0][0] + position[0][0]
        cY = cent[0]*resolution[0][0] + position[0][1]
        view_points.append([cX, cY])
        
        ###For Testing
        #mask = cv.circle(mask, (cent[1], cent[0]), 5, (255,255,255), 2)
        ###For Testing

    centroids = np.array(centroids)
    view_points = np.array(view_points)
    
    ###For Testing
    #image = np.where(mask == 255, 0, image)
    #mask[:,:,0] = 0
    #mask[:,:,1] = 0
    #image = np.where(image == 0, mask, image)
    #cv.imshow("Test",image)
    ###For Testing

    return centroids, view_points
"""
def scatter_test(mapa, view_points):
    resolution = mapa['resolution']
    width = int(mapa['width'])
    height = int(mapa['height'])
    data = mapa['map']
    position = mapa['position']
    
    group_available = [];
    group_barrier = [];
    group_unknown = [];

    f_mat = np.ones(data.shape,dtype=np.int64) * np.array([np.arange(height)]).T
    c_mat = np.ones(data.shape,dtype=np.int64) * np.array(np.arange(width))

    progress = 0
    last = 0
    
    f = f_mat[data == 0]
    c = c_mat[data == 0]
    for i in range(len(f)):
        progress += 1/(8*800)
        if last < int(progress):
            last = int(progress)
            print("Progress " + str(last) + "%")
        cX = c[i]*resolution[0][0] + position[0][0]
        cY = f[i]*resolution[0][0] + position[0][1]
        group_available.append([cX, cY])

    f = f_mat[data == 100]
    c = c_mat[data == 100]
    for i in range(len(f)):
        progress += 1/(8*800)
        if last < int(progress):
            last = int(progress)
            print("Progress " + str(last) + "%")
        cX = c[i]*resolution[0][0] + position[0][0]
        cY = f[i]*resolution[0][0] + position[0][1]
        group_barrier.append([cX, cY])

    f = f_mat[data == -1]
    c = c_mat[data == -1]
    for i in range(len(f)):
        progress += 1/(8*800)
        if last < int(progress):
            last = int(progress)
            print("Progress " + str(last) + "%")
        cX = c[i]*resolution[0][0] + position[0][0]
        cY = f[i]*resolution[0][0] + position[0][1]
        group_unknown.append([cX, cY])
    
    group_available = np.array(group_available)
    group_barrier = np.array(group_barrier)
    group_unknown = np.array(group_unknown)

    plt.scatter(group_available[:,0],group_available[:,1],c="blue")
    plt.scatter(group_barrier[:,0],group_barrier[:,1],c="black")
    plt.scatter(group_unknown[:,0],group_unknown[:,1],c="red")
    plt.scatter(view_points[:,0],view_points[:,1],c="green")
    plt.show()
"""
def run_test():   
    mapa = obtain_data('inflated.mat')
    t1 = time.time()

    image, limits = vecindades(mapa)
    t2 = time.time()
    print("Elapsed #1",t2 - t1)
    #print(limits)

    inicios, finales = interface_lines(image, limits)
    t3 = time.time()
    print("Elapsed #2",t3 - t2)
    #print(inicios)
    #print(finales)

    distances, inicios, finales = distance_discard(mapa, inicios, finales)
    t4 = time.time()
    print("Elapsed #3",t4-t3)
    #print(distances)
    #print(inicios)
    #print(finales)

    centroid, view_points = view_pointer(mapa, inicios, finales, distances, limits)
    t5 = time.time()
    print("Elapsed #4",t5-t4)
    print("Total Elapsed", t5-t1)
    #print(view_points)

    #scatter_test(mapa, view_points)
    #cv.waitKey(0)
    #cv.destroyAllWindows()

def solve_map(mapa):
    image, limits = vecindades(mapa)
    inicios, finales = interface_lines(image, limits)
    distances, inicios, finales = distance_discard(mapa, inicios, finales)
    centroid, view_points = view_pointer(mapa, inicios, finales, distances, limits)
    return view_points
