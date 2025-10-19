given a pair of lines, and a radius, find the circle center
https://stackoverflow.com/questions/51223685/create-circle-tangent-to-two-lines-with-radius-r-geometry/51235277#51235277
https://math.stackexchange.com/questions/797828/calculate-center-of-circle-tangent-to-two-lines-in-space

px1 = line1[0].x+line1[1].y*radius, 
py1 = line1[0].y-line1[1].x*radius, 
px2 = line2[0].x+line2[1].y*radius, 
py2 = line2[0].y-line2[1].x*radius, 
den = line1[1].x*line2[1].y-line2[1].x*line1[1].y, 
(if den is small, lines are nearly parallel)
k1 = (line2[1].y*(px2-px1)-line2[1].x*(py2-py1))/den, 
k2 = (line1[1].y*(px2-px1)-line1[1].x*(py2-py1))/den, 
cx = px1+k1*line1[1].x, 
cy = py1+k1*line1[1].y
or
cx = px2+k2*line2[1].x, 
cy = py2+k2*line2[1].y

to find the start and end points of the arc
tx1 = line1[0].x + k1*line1[1].x
ty1 = line1[0].y + k1*line1[1].y
tx2 = line2[0].x + k2*line2[1].x
ty2 = line2[0].y + k2*line2[1].y

