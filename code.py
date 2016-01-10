# coding: utf-8
# script pion.py hjyf
from Tkinter import *
import numpy
import numpy as np
from gurobipy import *
import timeit

####################################
############ FONCTIONS ############# 
####################################
gamma = 0.9
actions = [ ('T',(-2, -1)), ('R',(-1, -2)), ('F',(1, -2)), ('G',(2, -1)), ('H',(2, 1)), ('J',(1, 2)), ('U',(-1,2)), ('Y',(-2, 1)) ]

def case_possible(case):
    if not 0 <= case[0] < nblignes or not 0 <= case[1] < nbcolonnes or g[case] == -1:
        return False
    return True

def get_possible_destinations(case):
    cases = [ tuple(map(sum, zip(case, action[1]))) for action in actions]
    return [ c for c in cases if case_possible(c)]

def get_possible_adjacents(case):
    i,j = case
    return [ (i+x,j+y) for x in [-1,0,1] for y in [-1,0,1] if not(x==0 and y==0) and case_possible((i+x,j+y)) and (g[i+x][j+y]!=-1)]

def pl(g,OutputFlag=False):

    #Grille de recompenses
    r = np.zeros(g.shape)
    for i, gi in enumerate(g):
        for j, gij in enumerate(gi):
            if gij == 0:    r[i][j] = -2
            elif gij == -1: r[i][j] = None
            elif gij ==  2: r[i][j] = -1
            elif gij ==  3: r[i][j] = -1
    r[-1][-1] = 998

    # Creation d'un nouveau model
    model = Model("mogplex")
    model.setParam( 'OutputFlag', OutputFlag )

    # Declaration variables de decision
    #z = model.addVar(vtype=GRB.CONTINUOUS, lb=0, name="z")
    v = np.array( [ [ None if c == -1 else model.addVar(vtype=GRB.CONTINUOUS, lb=0, name="v"+str(i)+"_"+str(j)) for j,c in enumerate(ligne) ] for i,ligne in enumerate(g) ] )
    model.update() # maj du modele pour integrer les nouvelles variables

    # Definition de l'objectif
    obj = quicksum([vij for vij in v.reshape(nblignes*nbcolonnes) if vij != None])
    #obj.add(z, -1.0)
    model.setObjective(obj,GRB.MINIMIZE)

    # Definition des contraintes
    #model.addConstr(z <= quicksum(v[np.where(g==2)]), "zB")
    #model.addConstr(z <= quicksum(v[np.where(g==3)]), "zR")
    for i,vi in enumerate(v):
        for j,vij in enumerate(vi):
            if i+1==v.shape[0] and j+1==v.shape[1]:
                model.addConstr(vij == r[i][j],"goal")
            elif vij != None:
                dest = get_possible_destinations((i,j))
                for a,(xd,yd) in enumerate(dest):
                    expr = LinExpr(r[i][j])
                    adj = get_possible_adjacents((xd,yd))
                    for xa,ya in adj:
                        expr.add(v[xa][ya],gamma/16.)
                    expr.add(v[xd][yd],gamma*(1.-float(len(adj))/16.))
                    model.addConstr(vij >= expr, "v"+str(i)+"_"+str(j)+"_")

    # Resolution
    tmpExec = 0
    t = timeit.default_timer()
    model.optimize()
    tmpExec = timeit.default_timer() - t

    return np.array( [ [None if vij == None else vij.x for vij in l ]for l in v] )

def get_politique(v):
    politique = [['X' for j in range(nbcolonnes)]for i in range(nblignes)]
    for i, vi in enumerate(v):
        for j, vij in enumerate(vi):
            if g[i][j] != -1:
                cases = [ tuple(map(sum, zip((i,j), action[1]))) for action in actions]
                scores = [ v[c] if case_possible(c) else None for c in cases ]
                politique[i][j] = actions[np.argmax(scores)][0]
    return np.array(politique)

####################################
##########  INTERFACE ############## 
####################################

def initialize():
    global PosX,PosY,cost, globalcost
# position initiale du robot
    PosX = 20+10*zoom
    PosY = 20+10*zoom
    for k in range(5):
        cost[k]=0
# cout et affichage
    Canevas.coords(Pion,PosX -9*zoom, PosY -9*zoom, PosX +9*zoom, PosY +9*zoom)
    w.config(text='Cost = '+ str(globalcost))

def colordraw(g,nblignes,nbcolonnes):
    pmur=0.15 #0.15
    pblanc=0.55 #0.55
    pverte=0.1
    pbleue=0.1
    prouge=0.1
    pnoire=0.1
    for i in range(nblignes):
        for j in range(nbcolonnes):
            z=np.random.uniform(0,1)
            if z < pmur: c=-1
            elif z < pmur + pblanc: c=0
            elif z < pmur + pblanc + pverte: c=1
            elif z < pmur + pblanc + pverte + pbleue: c=2
            elif z < pmur + pblanc + pverte + pbleue + prouge: c=3
            else: c=4
            g[i,j]=c
    g[0,0]=0
    g[0,1]=0
    g[2,0]=0    
    g[nblignes-1,nbcolonnes-1]=0
    g[nblignes-2,nbcolonnes-1]=0
    g[nblignes-1,nbcolonnes-2]=0
    for i in range(nblignes):
        for j in range(nbcolonnes):          
            y =zoom*20*i+20
            x =zoom*20*j+20
            if g[i,j]>0:            
                Canevas.create_oval(x+zoom*(10-3),y+zoom*(10-3),x+zoom*(10+3),y+zoom*(10+3),width=1,outline=color[g[i,j]],fill=color[g[i,j]])
            else:
                if g[i,j]<0:
                    Canevas.create_rectangle(x, y, x+zoom*20, y+zoom*20, fill=myblack)
                    Canevas.create_rectangle(x, y, x+zoom*20, y+zoom*20, fill=myblack)
  
def Clavier(event):
    global PosX,PosY,cost,g, globalcost
    touche = event.keysym
    cj=(PosX-30)/(20*zoom)
    li=(PosY-30)/(20*zoom)
    changed=0
    # deplacement aleatoire en appuyant sur space
    if touche == 'space':
        t=np.random.randint(6)
        lettre = ['f','g','h','j','y','u',]
        touche=lettre[t]
    # deplacement (-2,1)
    if touche == 'y' and li>1 and cj < nbcolonnes-1 and g[li-2,cj+1]>-1:
        PosY -= zoom*20*2
        PosX += zoom*20 
        cost[g[li-2,cj+1]]+=1 
        changed=1
    # deplacement (-2,-1)
    if touche == 't' and li>1 and cj > 0 and g[li-2,cj-1]>-1:
        PosY -= zoom*20*2       
        PosX -= zoom*20                 
        cost[g[li-2,cj-1]]+=1
        changed=1
   # deplacement (-1,2)
    if touche == 'u' and li>0 and cj < nbcolonnes-2 and g[li-1,cj+2]>-1:
        PosY -= zoom*20        
        PosX += zoom*20*2       
        cost[g[li-1,cj+2]]+=1
        changed=1
    # deplacement (-1,-2)
    if touche == 'r' and li>0 and cj >1 and g[li-1,cj-2]>-1:
        PosY -= zoom*20
        PosX -= zoom*20*2           
        cost[g[li-1,cj-2]]+=1
        changed=1
     # deplacement (2,1)  
    if touche == 'h' and li<nblignes-2 and cj < nbcolonnes-1 and g[li+2,cj+1]>-1:
        PosY += zoom*20*2
        PosX += zoom*20 
        cost[g[li+2,cj+1]]+=1
        changed=1
    # deplacement (2,-1)
    if touche == 'g' and li<nblignes-2 and cj > 0 and g[li+2,cj-1]>-1:
        PosY += zoom*20*2       
        PosX -= zoom*20                 
        cost[g[li+2,cj-1]]+=1
        changed=1
   # deplacement (1,2)
    if touche == 'j' and li<nblignes-1 and cj < nbcolonnes-2 and g[li+1,cj+2]>-1:
        PosY += zoom*20        
        PosX += zoom*20*2       
        cost[g[li+1,cj+2]]+=1
        changed=1
    # deplacement (1,-2)
    if touche == 'f' and li<nblignes-1 and cj >1 and g[li+1,cj-2]>-1:
        PosY += zoom*20
        PosX -= zoom*20*2           
        cost[g[li+1,cj-2]]+=1 
        changed=1


# La variable alea =1 si on veut des effets aleatoires sinon les transitions sont deterministes
    #On ajoute un effet aleatoire dans les transitions
    if alea==1 and changed==1:
        t=np.random.uniform(0,1)    
        if t>0.5:
            d=np.random.randint(8)
            dli=0
            if d== 0 or d==1 or d==2:
                dli=-1
            if d== 4 or d==5 or d==6:
                dli==1
            dcj=0
            if d==0 or d==7 or d==6:
                dcj=-1
            if d==2 or d==3 or d==4:
                dcj=1    
        # l'effet aleatoire est applique s'il cree un deplacement sur une case admissible     
            NewPosY = PosY+zoom*20*dli
            NewPosX = PosX+zoom*20*dcj        
            newcj=(NewPosX-30)/(20*zoom)
            newli=(NewPosY-30)/(20*zoom)
            print('d',dli,dcj)
            if newli>=0 and newcj>=0 and newli<=nblignes-1 and newcj<=nbcolonnes-1 and g[newli,newcj]>-1:
                PosY=NewPosY
                PosX=NewPosX            
            
# on dessine le pion a sa nouvelle position
    Canevas.coords(Pion,PosX -9*zoom, PosY -9*zoom, PosX +9*zoom, PosY +9*zoom)       
    globalcost=0    
    for k in range(5):
        globalcost+=cost[k]*weight[k]
    w.config(text='Cost = '+ str(globalcost))  

Mafenetre = Tk()
Mafenetre.title('MDP')

zoom=2

alea = 0 #transitions aleatoires si alea =1 sinon mettre alea=0

#taille de la grille
nblignes=10
nbcolonnes=15
 
globalcost=0

# Creation d'un widget Canvas (pour la grille)
Largeur = zoom*20*nbcolonnes+40
Hauteur = zoom*20*nblignes+40
 
# valeurs de la grille
g= np.zeros((nblignes,nbcolonnes), dtype=numpy.int)
cost= np.zeros(5, dtype=numpy.int)
weight= np.zeros(5, dtype=numpy.int)
weight[0] = 1
weight[1] = 10
weight[2] = 20
weight[3] = 30
weight[4] = 40

# def des couleurs
myred="#D20B18"
mygreen="#25A531"
myblue="#0B79F7"
mygrey="#E8E8EB"
myyellow="#F9FB70"
myblack="#2D2B2B"
mywalls="#5E5E64"
mywhite="#FFFFFF"
color=[mywhite,mygreen,myblue,myred,myblack]

# ecriture du quadrillage et coloration
Canevas = Canvas(Mafenetre, width = Largeur, height =Hauteur, bg =mywhite)
for i in range(nblignes+1):
    ni=zoom*20*i+20
    Canevas.create_line(20, ni, Largeur-20,ni)
for j in range(nbcolonnes+1):
    nj=zoom*20*j+20
    Canevas.create_line(nj, 20, nj, Hauteur-20)
colordraw(g,nblignes,nbcolonnes)



v = pl(g)
p = get_politique(v)
test = np.array( [[ None if cel == None else int(cel) for cel in ligne]for ligne in v] )
print(test)
for i,lin in enumerate(g):
    for j,col in enumerate(lin):
        y = j*20*zoom+20
        x = i*20*zoom+20
        if g[i][j] != -1:
            rec = Canevas.create_rectangle(y, x, y+zoom*20, x+zoom*20)
            Canevas.tag_lower(rec)
            Canevas.create_text(y +10, x +10, text=p[i][j])



 
Canevas.focus_set()
Canevas.bind('<Key>',Clavier)
Canevas.pack(padx =5, pady =5)

PosX = 20+10*zoom
PosY = 20+10*zoom

# Creation d'un widget Button (bouton Quitter)
Button(Mafenetre, text ='Restart', command = initialize).pack(side=LEFT,padx=5,pady=5)
Button(Mafenetre, text ='Quit', command = Mafenetre.destroy).pack(side=LEFT,padx=5,pady=5)

w = Label(Mafenetre, text='Cost = '+str(globalcost),fg=myblack,font = "Verdana 14 bold")
w.pack() 

Pion = Canevas.create_oval(PosX-10,PosY-10,PosX+10,PosY+10,width=1,outline='black')

initialize()

Mafenetre.mainloop()


'''
nblignes=10
nbcolonnes=10

g = np.zeros((nblignes,nbcolonnes), dtype=np.int)

pmur=0.15
pblanc=0.55
pverte=0.1
pbleue=0.1
prouge=0.1
pnoire=0.1
for i in range(nblignes):
    for j in range(nbcolonnes):
        z=np.random.uniform(0,1)
        if z < pmur: c=-1
        elif z < pmur + pblanc: c=0
        elif z < pmur + pblanc + pverte: c=1
        elif z < pmur + pblanc + pverte + pbleue: c=2
        elif z < pmur + pblanc + pverte + pbleue + prouge: c=3
        else: c=4
        g[i,j]=c
g[0,0]=0
g[0,1]=0
g[2,0]=0
g[nblignes-1,nbcolonnes-1]=0
g[nblignes-2,nbcolonnes-1]=0
g[nblignes-1,nbcolonnes-2]=0


def cost(case):
    case_type = g[case]

    if case_type == 0:
        return -1
    elif case_type == 1:
        return -11
    elif case_type == 2:
        return -21
    elif case_type == 3:
        return -31
    return NULL

vt = np.zeros((nblignes,nbcolonnes))
vt[-1][-1]=1000
vtplusun = 0
i = 0
while True:
    i += 1
    if i % 20 == 0: print(np.array(vt))
    vtplusun = [[max([cost(dest)+gamma*vt[dest[0]][dest[1]] for dest in get_possible_destinations((i,j))]) for i in range(nblignes)] for j in range(nbcolonnes)]
    if np.array_equal(vt, vtplusun):
        break
    vt=vtplusun

v2 = [[max([cost(dest) for dest in get_possible_destinations((i,j))]) for i in range(nblignes)] for j in range(nbcolonnes)]
'''
