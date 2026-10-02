"""Replaceable local detector; output original-frame corners, centres and confidence."""
import cv2
import numpy as np


def detect(gray, threshold=120, min_area_fraction=.008, max_area_fraction=.75):
    if gray.ndim != 2 or gray.dtype != np.uint8:
        raise ValueError("detector expects mono8 original capture pixels")
    h,w=gray.shape
    _,mask=cv2.threshold(gray,threshold,255,cv2.THRESH_BINARY_INV)
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    output=[]
    for contour in contours:
        area=cv2.contourArea(contour)
        if not min_area_fraction*w*h <= area <= max_area_fraction*w*h:
            continue
        poly=cv2.approxPolyDP(contour,.025*cv2.arcLength(contour,True),True)
        if len(poly)!=4 or not cv2.isContourConvex(poly):
            continue
        points=poly.reshape(4,2).astype(float)
        # Require complete rectangle; clipped/edge-truncated panels are not centres.
        if np.min(points[:,0]) < 3 or np.max(points[:,0]) > w-4 or np.min(points[:,1]) < 3 or np.max(points[:,1]) > h-4:
            continue
        rect=cv2.minAreaRect(contour)
        a,b=rect[1]
        if min(a,b)<10 or max(a,b)/min(a,b)>4:
            continue
        rectangularity=area/(a*b)
        if rectangularity < .78:
            continue
        # Projective rectangle centre = diagonal intersection, not mean corners.
        centre=diagonal_centre(points)
        output.append({"corners":points.tolist(),"center":centre.tolist(),
                       "score":min(1.,rectangularity),"width":w,"height":h})
    return sorted(output,key=lambda x:-x["score"])[:8]


def diagonal_centre(p):
    p=np.asarray(p,dtype=float)
    a,b=p[0],p[2]
    c,d=p[1],p[3]
    system=np.column_stack((b-a,-(d-c)))
    if abs(np.linalg.det(system)) < 1e-6:
        raise ValueError("degenerate panel quadrilateral")
    t,_=np.linalg.solve(system,c-a)
    return a+t*(b-a)


def optical_ray(uv,cal):
    fx,fy,cx,cy=cal["intrinsics"]
    k=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
    p=cv2.undistortPoints(np.array(uv,dtype=float).reshape(1,1,2),k,np.array(cal["distortion"]))[0,0]
    return np.array([p[0],p[1],1.])
