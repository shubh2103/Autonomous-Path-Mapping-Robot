import gym
import pixelate_arena
import time
import pybullet as p
import pybullet_data
import cv2
import os
import numpy as np
from queue import PriorityQueue
import cv2.aruco as aruco

src=-1
n=13
m=27
inf =10000
initmatrix = np.full([n,m],0,dtype=int)
initmatrix2 = np.full([n,m],0,dtype=int)
initmatrix3 = np.full([n,m],0,dtype=int)
graph={}
cases={}
#green = 4
#purple = 3
#yellow = 2
#white,red and pink =1

spidey1=(-1,-1)
spidey2=(-1,-1)
goblin=(-1,-1)
gob_anti=(-1,-1)
electro=(-1,-1)
ele_anti=(-1,-1)
sandman=(-1,-1)
san_anti=(-1,-1)
pink=[]
#image croping and resizing 

def imgCrop(img):
    img = img[40:560,10:590]
    cv2.imshow("abc",img)
    img =cv2.resize(img,(2600,1300))
    return img

#to get outer colour of the shape in blue colour(villains)

def outercol(res) :
    a,b,c = res[50,98]
    if(a==30 and b==125 and c==33):
        return 4 #green
    elif(a==99 and b==11 and c==76):
        return 3 #purple
    else :
        return 2 #yellow   

# geting shape 
def getshape(res) :
      res1 = cv2.cvtColor(res,cv2.COLOR_BGR2HSV)
      lower_blue = np.array([78,158,124])
      upper_blue = np.array([138,255,255])
      mask = cv2.inRange(res1,lower_blue,upper_blue)
      #cv2.imshow("mask",mask)
      contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      for cntr in contours:
          area=cv2.contourArea(cntr)
          if area>=300:
              per=cv2.arcLength(cntr, closed=True)
              approx=cv2.approxPolyDP(cntr, epsilon=0.036*per, closed=True)
              #print(len(approx))
              return(approx)


#creating weighted graph

def createGraph(initmatrix):
    m=27
    n=13
    for i in range(n):
        for j in range(1,m-1):
            v=i*m+j
            graph[v]=[]
            if initmatrix[i][j]==inf:
                continue
            if i>0 :
                if j>0:
                    if initmatrix[i-1][j-1]!=inf:
                        graph[v].append((i-1)*m+j-1)
                if j<26:
                    if initmatrix[i-1][j+1]!=inf:
                        graph[v].append((i-1)*m+j+1)
            if i<12:
                if j>0:
                    if initmatrix[i+1][j-1]!=inf:
                        graph[v].append((i+1)*m+j-1)
                if j<26:
                    if initmatrix[i+1][j+1]!=inf:
                        graph[v].append((i+1)*m+j+1)  
            if j>1:
                if initmatrix[i][j-2]!=inf:
                    graph[v].append((i)*m+j-2)
            if j<25 :
                if initmatrix[i][j+2]!=inf:
                    graph[v].append((i*m)+j+2)                

# recognising colour of matrix's cells

def getcolor(res) :
    a,b,c = res[50,50]
    #print(res[70,60])
    if(a>=225 and b>=225 and c>=225):
        return 1 #white
    elif(a==99 and b==11 and c==76):
        return 3 #purple
    elif(a==30 and b==125 and c==33):
        return 4 #green
    elif(a<=3 and b<=3 and c<=3):
        return -1 #black
    elif(a==0 and b==182 and c==227):
        return 2 #yellow
    elif(a==152 and b==63 and c==1):
        return 10 #blue
    elif(a==0 and b==18 and c==193):
        return 0 #red
    else:
        return 12 #pink

# making cost matrix

def initialmatrix(img) :
     global spidey1
     global spidey2
     global goblin
     global gob_anti
     global electro
     global ele_anti
     global sandman
     global san_anti
     global pink1
     global pink2
     global pink3
     spidey1=(-1,-1)
     for i in range(n):
         for j in range(m):
             if(j==0 or j==26):
                 initmatrix[i][j]=inf
                 initmatrix2[i][j]=initmatrix[i][j]
                 continue
             if (i%2==0):
               if(j%2==0):
                 initmatrix[i][j]=inf
                 initmatrix2[i][j]=inf
                 continue
             if (i%2==1):
                 if(j%2==1):
                     initmatrix[i][j]=inf
                     initmatrix2[i][j]=inf
                     continue
             if(j==26):
                 initmatrix[i][j]=inf
                 initmatrix2[i][j]=inf
                 continue
             res = img[i*100 : (i+1)*100,j*100-50 : (j+1)*100-50]
             col = getcolor(res)
             if col==10 :
                     col1 = outercol(res)   
                     if col1==4:
                         #cv2.imshow("gob",res)
                         goblin=(i,j)
                         initmatrix[i][j]=inf
                         initmatrix2[i][j]=4
                     elif col1==2:
                         #cv2.imshow("ele",res)
                         electro = (i,j)
                         initmatrix[i][j]=inf
                         initmatrix2[i][j]=2
                     else :
                         #cv2.imshow("san",res)
                         sandman=(i,j)
                         initmatrix[i][j]=inf
                         initmatrix2[i][j]=3 
             elif col==0 :
                 if(i==6 and j==13):
                     initmatrix[i][j]=1
                     initmatrix2[i][j]=1
                     continue
                 elif spidey1==(-1,-1):
                     spidey1=(i,j)
                 else :
                     spidey2=(i,j)
                 initmatrix[i][j]=1
                 initmatrix2[i][j]=1
             elif col==-1:
                 initmatrix[i][j]=inf
                 initmatrix2[i][j]=inf
             elif col==12:
                  pink.append((i,j))        
                  initmatrix[i][j]=1
                  initmatrix2[i][j]=1
             else :
                 initmatrix[i][j]=col
                 initmatrix2[i][j]=col
             initmatrix[i][j+1]=inf
             initmatrix2[i][j+1]=inf
          
# dijktras algorithm

def dijktras(src,dest,cost,n,m):
    q=PriorityQueue()
    par={}
    par[src]=src
    (srcx,srcy)=src
    dist=np.zeros((n,m))
    for i in range(n):
        for j in range(m):
            dist[i][j]=inf
    dist[srcx][srcy]=0
    destx,desty=dest
    q.put((0,src))
    d=[1,-1,0]
    while not q.empty():
        wt,(x,y)=q.get()
        for i in range(len(graph[x*m+y])):
            v=graph[x*m+y][i]
            dx=v//m
            dy=v%m
            if(dist[dx][dy]>dist[x][y]+cost[dx][dy]):
                dist[dx][dy]=dist[x][y]+cost[dx][dy]
                par[(dx,dy)]=(x,y)
                q.put((dist[dx][dy],(dx,dy)))


    path=[]
    (destx,desty)=dest
    path.append(dest)
    #print(dest)
    while destx!=srcx or desty!=srcy:
        (destx,desty)=par[(destx,desty)]
        #print((destx,desty))
        path.append((destx,desty))

    path.reverse()
    for i in range(len(path)):
        x,y=path[i]
        if(i!=len(path)-1):
            print("({} ,{})->".format(x,y),end='')
        else:
            print("({} ,{})".format(x,y))

    return path

# getting corner points of aruco marker(bot)

def get_corner(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    img = aruco.drawDetectedMarkers(img, corners, borderColor=(0, 0, 255))
    if ids is None:
        return([])
    corners=np.array(corners)
    corner=corners[0]
    corner=corner.reshape((corner.shape[1],corner.shape[2]))
    return(corner)    

# getting position of bot in form of i,j (indexes of matrix)

def bot_pos(img) :
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    img = aruco.drawDetectedMarkers(img, corners, borderColor=(0, 0, 255))
    if ids is None:
        return([])
    corners=np.array(corners)
    corner=corners[0]
    corner=corner.reshape((corner.shape[1],corner.shape[2]))
    x1=(corner[0][0]+corner[2][0])//2
    y1=(corner[0][1]+corner[2][1])//2
    x1 =int(x1//100)
    y1 = int(y1//100)
    if(x1%2==0 and y1%2==0):
        x1+=1
    if(x1%2==1 and y1%2==1):
        x1+=1
    return((x1,y1))

# deciding direction of motion

def movement(src,corner,dest,img) :
    x,y=src
    #print(x,y,dest[0],dest[1])
    #print(corner[0][0],corner[0][1],corner[3][0],corner[3][1])
    vxreq=(dest[0]-x)
    vyreq=(dest[1]-y)
    modv=np.sqrt(vxreq**2+vyreq**2)
    vxreq=vxreq/modv
    vyreq=vyreq/modv
    if(len(corner)==0):
        return("left")
    botvx=(corner[0][0]-corner[3][0])
    botvy=(corner[0][1]-corner[3][1])
    mod=np.sqrt(botvx**2+botvy**2)
    botvx=botvx/mod
    botvy=botvy/mod
    botvec=complex(botvx,botvy)
    vec=complex(vxreq,vyreq)
    angle=np.angle(botvec/vec,deg=True)
    #print("angle :",angle)
    if(-7.2<=angle and angle<=7.2):
        return("straight")
    elif(angle>=7.2):
        return("left")
    else:
        return("right")

# converting indexes to coordinates

def coordi_2_pix(index) :
    x,y = index
    x = x*100
    y = y*100
    return (x,y)
    # if(x%2==0):
    #     if(y%2==0):
    #         a= x*100
    #         b = (y*100 + (y+1)*100)//2 + 50
    #         return (a,b)
    #     else:
    #         a = x*100
    #         b = (y*100 + (y-1)*100)//2 + 50
    #         return (a,b)
    # else :
    #     if(y%2==0):
    #         a = x*100
    #         b = (y*100 + (y-1)*100)//2 + 50
    #         return (a,b)
    #     else :
    #         a= x*100
    #         b = (y*100 + (y+1)*100)//2 + 50
    #         return (a,b)                       


def stop():
    k=0
    while k<20 :
        k+=1
        p.stepSimulation()
        env.move_husky(0.0,0.0,0.0,0.0)

def turn_left():
    k=0
    while k<100 :
        k+=1
        if k%30==0:
            img = env.camera_feed()
            img = imgCrop(img)
        p.stepSimulation()
        env.move_husky(-4.0,4.0,-4.0,4.0)

def turn_right():
    k=0
    while k<100 :
        k+=1
        if k%30==0:
            img = env.camera_feed()
            img = imgCrop(img)
        p.stepSimulation()
        env.move_husky(4.0,-4.0,4.0,-4.0)


def move_frwd():
    k=0
    while k<100 :
        k+=1
        if k%30==0:
            img = env.camera_feed()
            img = imgCrop(img)
        p.stepSimulation()
        env.move_husky(6.0,6.0,6.0,6.0)

def distance(dest,src) :
    y,x=src
    return(np.sqrt((y-dest[0])**2+(x-dest[1])**2))

def move(dest) :
    while True :
        img = env.camera_feed()
        img = imgCrop(img)
        corner = get_corner(img)
        if(len(corner)==0):
            turn_left()
            stop()
            continue  
        y1=(corner[0][0]+corner[2][0])//2
        x1=(corner[0][1]+corner[2][1])//2
        src= (y1,x1)
        
        dist = distance(dest,src)
        #print("dist",dist)
        if(dist<20.0):
            stop()
            #print("sssss")
            break
        move = movement(src,corner,dest,img)
        if(move=="left"):
            turn_left()
            stop()
        elif(move=="right"):
            turn_right()
            stop()
        else :
            move_frwd()
            stop()

#unlocking antidote
def unlock_anti():
    x=0
    while True:
        p.stepSimulation()
        if x==1000:
           env.unlock_antidotes()
           time.sleep(2)
           return 1
        x+=1

## getting coordinates of cure after unlocking them
def cure_1(img):
    global san_anti
    global gob_anti
    global ele_anti
    k=0
    while k<3:
        i,j=pink[k]
        res = img[i*100:(i+1)*100,j*100-50:(j+1)*100-50]
        #cv2.imshow("res",res)
        appr = getshape(res)
        if(len(appr) is 3):
            san_anti=(i,j)
        elif (len(appr) is 4):
            gob_anti=(i,j)
        else:
            ele_anti=(i,j)
        k+=1    

### calculating cost of path
def cal_cost(case_1):
    k=0
    global initmatrix
    global initmatrix2
    global initmatrix3
    createGraph(initmatrix)
    cost=0
    while k<3:
        src=case_1[k]
        end_1 = case_1[k+1]
        path_n =  dijktras(src,end_1,initmatrix,13,27)
        for i in range(len(path_n)-1):
            i,j=path_n[i]
            cost += initmatrix[i][j]
        k+=1
    k=3
    while k<6:
        src=case_1[k]
        end_1 = case_1[k+1]
        l,q = end_1
        initmatrix[l][q]=initmatrix2[l][q]
        createGraph(initmatrix)
        path_n =  dijktras(src,end_1,initmatrix,13,27)
        for i in range(len(path_n)-1):
            i,j=path_n[i]
            cost += initmatrix[i][j]
        k+=1
    i,j=case_1[len(case_1)-1]

    cost += initmatrix2[i][j]
    initmatrix=initmatrix3
    return cost   

#main

if __name__=="__main__":
    x1=100
    y1=100
    env = gym.make("pixelate_arena-v0")
    #time.sleep(10)
    env.remove_car()
    img = env.camera_feed()
    #cv2.imshow("img", img)
    img = imgCrop(img)
    print(img.shape)
    initialmatrix(img)
    createGraph(initmatrix)
    initmatrix3=initmatrix
    print(initmatrix)
    env.respawn_car()
    img = env.camera_feed()
    img = imgCrop(img)
    src=(6,13)
    print("src",src)
    print("spidey1",spidey1)
    print("spidey2",spidey2)
    print("electro",electro)
    print("elec_anti",ele_anti)
    print("gobl",goblin)
    print("gob_an",gob_anti)
    print("san",sandman)
    print("san_ant",san_anti)
    #cv2.waitKey(0)
    k=0
    #while True:
   # path = dijktras((10,17),(6,9),initmatrix,13,27)
   # print(path)
    end_pt = spidey1
    end_pt1 = spidey2
    path1 = dijktras(src,end_pt,initmatrix,13,27)
    path2 = dijktras(src,end_pt1,initmatrix,13,27)
    if(len(path1)>len(path2)):
     end_pt=end_pt1
     end_pt1=spidey1
    path = dijktras(src,end_pt,initmatrix,13,27)
    for i in range(len(path)-1) :
        y,x = path[i]
        dest = [x*100,y*100+50]
        move(dest)
        #print("yipiiii....")
        #print(dest)
    y,x = path[len(path)-1]
    dest = [x*100,y*100+50]
    move(dest) 
    #unlock_anti()
    src = end_pt
    path = dijktras(src,end_pt1,initmatrix,13,27)
    for i in range(len(path)-1) :
         y,x = path[i]
         dest = [x*100,y*100+50]
         move(dest)
         #print("yipiiii....")
         #print(dest)

    y,x = path[len(path)-1]
    dest = [x*100,y*100+50]
    move(dest) 
    unlock_anti()
    src=end_pt1
    print(end_pt1)
    img = env.camera_feed()
    img = imgCrop(img)
    cure_1(img)
    print("san",san_anti)
    print("ele",ele_anti)
    print("gob",gob_anti)
    #case 1
    cases[1]=[]
    cases[1].append(src)
    cases[1].append(gob_anti)
    cases[1].append(san_anti)
    cases[1].append(ele_anti)
    cases[1].append(goblin)    
    cases[1].append(sandman)
    cases[1].append(electro)
    #case 2
    cases[2]=[]
    cases[2].append(src)
    cases[2].append(gob_anti)
    cases[2].append(ele_anti)
    cases[2].append(san_anti)
    cases[2].append(goblin)    
    cases[2].append(electro)
    cases[2].append(sandman)   
    #case 3
    cases[3]=[]
    cases[3].append(src)
    cases[3].append(san_anti)
    cases[3].append(gob_anti)
    cases[3].append(ele_anti)
    cases[3].append(sandman)    
    cases[3].append(goblin)
    cases[3].append(electro)
    #case 4
    cases[4]=[]
    cases[4].append(src)
    cases[4].append(san_anti)
    cases[4].append(ele_anti)
    cases[4].append(gob_anti)
    cases[4].append(sandman)    
    cases[4].append(electro)
    cases[4].append(goblin)   
    #case 5
    cases[5]=[]
    cases[5].append(src)
    cases[5].append(ele_anti)
    cases[5].append(gob_anti)
    cases[5].append(san_anti)
    cases[5].append(electro)    
    cases[5].append(goblin)
    cases[5].append(sandman) 
    #case 6
    cases[6]=[]
    cases[6].append(src)
    cases[6].append(ele_anti)
    cases[6].append(san_anti)
    cases[6].append(gob_anti)
    cases[6].append(electro)    
    cases[6].append(sandman)
    cases[6].append(goblin)   
    minimum=inf
    min_case = -1
    k=1
    while k<7:
        cost = cal_cost(cases[k])
        if(cost<minimum):
            minimum=cost
            min_case=k
        k+=1
    k=0
    t=min_case
    print(t)
    createGraph(initmatrix)
    while k<3:
         src=cases[t][k]
         end = cases[t][k+1]
         path = dijktras(src,end,initmatrix,13,27)
         for i in range(len(path)-1) :
           y,x = path[i]
           dest = [x*100,y*100+50]
           move(dest)
          #print("yipiiii....")
          #print(dest)
         y,x = path[len(path)-1]
         dest = [x*100,y*100+50]     
         move(dest) 
         k+=1 
    while k<6:
         src=cases[t][k]
         end = cases[t][k+1]
         l,q = end
         initmatrix[l][q]=initmatrix2[l][q]
         createGraph(initmatrix)
         path = dijktras(src,end,initmatrix,13,27)
         for i in range(len(path)-1) :
           y,x = path[i]
           dest = [x*100,y*100+50]
           move(dest)
          #print("yipiiii....")
          #print(dest)
         y,x = path[len(path)-1]
         dest = [x*100,y*100+50]     
         move(dest) 
         k+=1         
    cv2.waitKey(0)
  #  while True:
  #      p.stepSimulation()