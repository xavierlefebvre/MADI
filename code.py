# coding: utf-8
# script pion.py hjyf
from Tkinter import *
import numpy as np
from gurobipy import *
import timeit

####################################
############ FONCTIONS ############# 
####################################
gamma = 0.9
actions = {'t':(-2, -1), 'r':(-1, -2), 'f':(1, -2), 'g':(2, -1), 'h':(2, 1), 'j':(1, 2), 'u':(-1,2), 'y':(-2, 1)}
actionsNames = ['t','r','f','g','h','j','u','y']
reversed_action = {'t':'h', 'r':'j', 'f':'u', 'g':'y', 'h':'t', 'j':'r', 'u':'f', 'y':'g'}

def case_possible(case):
    i,j = case
    return i>= 0 and i<g.shape[0] and j>=0 and j<g.shape[1] and g[i,j] != -1

def get_possible_actions(case):
    return np.array([ action for action, move in actions.items() if case_possible( tuple(map(sum,zip(case,move))))])

def get_possible_adjacents(case,dist=1):
    var = np.arange(-dist,dist+1)
    i,j = case
    return np.array( [ (i+x,j+y) for x in var for y in var if not(x==0 and y==0) and case_possible((i+x,j+y)) and (g[i+x][j+y]!=-1)])

def get_possible_dests(case,action):
    dest = map(sum,zip(case,actions[action]))
    adj_dest = get_possible_adjacents(dest)
    return np.array([ [np.array(dest),1.-(len(adj_dest)/16.)]] + [ [adj,1./16.] for adj in adj_dest])

def pl(g,OutputFlag=False):

    #Grille de recompenses bleu
    Rb = np.where(g==2,-1,-2)
    Rb[-1,-1] = 998
    #Grille de recompenses rouge
    Rr = np.where(g==3,-1,-2)
    Rr[-1,-1] = 998

    # Creation d'un nouveau model
    model = Model("dual")
    model.setParam( 'OutputFlag', OutputFlag )

    # Declaration variables de decision
    Z = model.addVar(vtype=GRB.CONTINUOUS, lb=-10000000, name="Z")
    X = np.array( [[ {} if case != -1 else None for case in ligne] for ligne in g] )
    D = np.copy(X)
    for x in range(g.shape[0]):
        for y in range(g.shape[1]):
            if g[x,y] != -1:
                possible_actions = [] if x==g.shape[0]-1 and y==g.shape[1]-1 else get_possible_actions((x,y))
                X[x,y] = dict( [ (action, model.addVar(vtype=GRB.CONTINUOUS, name="X(%d,%d)_%s"%(x,y,action))) for action in possible_actions ])
                D[x,y] = dict( [ (action, model.addVar(vtype=GRB.BINARY, name="D(%d,%d)_%s"%(x,y,action))) for action in possible_actions ])
    model.update()

    nbL,nbC = X.shape
    sspa = np.zeros((nbL,nbC,nbL,nbC,len(actions)))
    for x in range(X.shape[0]):
        for y in range(X.shape[1]):
            if X[x,y] != None and not(np.array_equal([x,y],[X.shape[0]-1,X.shape[1]-1])):
                for action in get_possible_actions((x,y)):
                    for dest,proba in get_possible_dests((x,y),action):
                        sspa[x,y][dest[0],dest[1]][actionsNames.index(action)] = proba

    # Definition de l'objectif
    model.setObjective(Z,GRB.MAXIMIZE)
    model.update()

    # Definition des contraintes
    contrainteBleue = LinExpr()
    contrainteRouge = LinExpr()
    for x in range(g.shape[0]):
        for y in range(g.shape[1]):
            if g[x,y] != -1 and X[x,y] != {}:
                model.addConstr( quicksum(D[x,y].values()) <= 1,"P0(%d,%d)"%(x,y)) # contraintes pour forcer des politiques pures
                som = LinExpr()
                for action in X[x,y].keys():
                    model.addConstr( (1 - gamma)*X[x,y][action] <= D[x,y][action],"P1(%d,%d)_%s"%(x,y,action)) # contraintes pour forcer des politiques pures
                    dest = tuple(map(sum,zip((x,y),actions[action])))
                    adj = get_possible_adjacents(dest)
                    coefB = ( 1. - len(adj) / 16.) * Rb[dest] + (1./16.) * np.sum(Rb[adj[:,0],adj[:,1]])
                    coefR = ( 1. - len(adj) / 16.) * Rr[dest] + (1./16.) * np.sum(Rr[adj[:,0],adj[:,1]])
                    contrainteBleue.add(X[x,y][action],coefB)
                    contrainteRouge.add(X[x,y][action],coefR)
                    adj_xy = get_possible_adjacents((x,y))
                    if dest != (g.shape[0]-1,g.shape[1]-1):
                        for xp,array in enumerate(sspa[:,:,x,y,:]):
                            for yp,ligne in enumerate(array):
                                for a,proba in enumerate(ligne):
                                    if proba != 0:
                                        som.add(X[xp,yp][actionsNames[a]],proba)
                model.addConstr( (quicksum(X[x,y].values()) -gamma * som) == 1.,"C(%d,%d)"%(x,y)) # contraintes
    model.addConstr( Z <= contrainteBleue,'LB') # contrainte de linearisation bleue
    model.addConstr( Z <= contrainteRouge,'LR') # contrainte de linearisation rouge
    model.update()

    # Resolution
    tmpExec = 0
    t = timeit.default_timer()
    model.optimize()
    model.write("out.lp")
    tmpExec = timeit.default_timer() - t

    politique = np.chararray(g.shape)
    for x in range(g.shape[0]):
        for y in range(g.shape[1]):
            for action, d in D[x,y].items():
                if d.x == 1:
                    politique[x,y] = action
                    break

    return politique

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
    pmur=0.1 #0.15
    pblanc=0.5 #0.55
    pverte=0
    pbleue=0.2
    prouge=0.2
    pnoire=0
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
        touche=p[li,cj]
        changed=1
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

alea = 1 #transitions aleatoires si alea =1 sinon mettre alea=0

#taille de la grille
nblignes=10
nbcolonnes=15
 
globalcost=0

# Creation d'un widget Canvas (pour la grille)
Largeur = zoom*20*nbcolonnes+40
Hauteur = zoom*20*nblignes+40
 
# valeurs de la grille
g= np.zeros((nblignes,nbcolonnes), dtype=np.int)
cost= np.zeros(5, dtype=np.int)
weight= np.zeros(5, dtype=np.int)
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





p = pl(g)

for i,lin in enumerate(g):
    for j,col in enumerate(lin):
        y = j*20*zoom+20
        x = i*20*zoom+20
        if g[i][j] != -1:
            rec = Canevas.create_rectangle(y, x, y+zoom*20, x+zoom*20)
            Canevas.tag_lower(rec)
            Canevas.create_text(y +10, x +10, text=p[i,j])



 
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

Pion = Canevas.create_oval(PosX-10,PosY-10,PosX+10,PosY+10,width=1,outline='black',fill='yellow')
Canevas.lower(Pion)

initialize()

Mafenetre.mainloop()
