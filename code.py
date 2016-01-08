# coding: utf-8

import numpy as np
from gurobipy import *
import timeit

nblignes=10
nbcolonnes=10
gamma = 0.9
actions = {'T': (-2, -1),
           'R': (-1, -2),
           'F': (1, -2),
           'G': (2, -1),
           'H': (2, 1),
           'J': (1, 2),
           'U': (-1,2),
           'Y': (-2, 1)}
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

def case_possible(case):
    if not 0 <= case[0] < nblignes or not 0 <= case[1] < nbcolonnes or g[case] == -1:
        return False
    return True

def get_possible_destinations(case):
    cases = [ tuple(map(sum, zip(case, coord))) for coord in actions.values()]
    return [ c for c in cases if case_possible(c)]

def get_possible_adjacents(case):
    i,j = case
    return [ (i+x,j+y) for x in [-1,0,1] for y in [-1,0,1] if not(x==0 and y==0) and case_possible((i+x,j+y)) and (g[i+x][j+y]!=-1)]

'''
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
    z = model.addVar(vtype=GRB.CONTINUOUS, lb=0, name="z")
    v = np.array( [ [ None if c == -1 else model.addVar(vtype=GRB.CONTINUOUS, lb=0, name="v"+str(i)+"_"+str(j)) for j,c in enumerate(ligne) ] for i,ligne in enumerate(g) ] )
    model.update() # maj du modele pour integrer les nouvelles variables

    # Definition de l'objectif
    obj = quicksum([vij for vij in v.reshape(nblignes*nbcolonnes) if vij != None])
    obj.add(z, -1.0)
    #obj = quicksum(model.getVars())
    model.setObjective(obj,GRB.MINIMIZE)

    # Definition des contraintes
    model.addConstr(z <= quicksum(v[np.where(g==2)]), "zB")
    model.addConstr(z <= quicksum(v[np.where(g==3)]), "zR")
    for i,vi in enumerate(v):
        for j,vij in enumerate(vi):
            if vij != None:
                dest = get_possible_destinations((i,j))
                for a,(xd,yd) in enumerate(dest):
                    expr = LinExpr(r[i][j])
                    adj = get_possible_adjacents((xd,yd))
                    for xa,ya in adj:
                        expr.add(v[xa][ya],gamma/16.)
                    expr.add(v[xd][yd],gamma*(1.-float(len(adj))/16.))
                    model.addConstr(vij >= expr, "v"+str(i)+"_"+str(j)+"_"+[action for action, case in actions.iteritems() if case == (xd-i,yd-j)][0])

    
    # Resolution
    tmpExec = 0
    t = timeit.default_timer()
    model.optimize()
    model.write("out.lp")
    tmpExec = timeit.default_timer() - t

    return np.array( [ [None if vij == None else vij.x for vij in l ]for l in v] )

def get_politique(v):
    politique = [['x' for j in range(nbcolonnes)]for i in range(nblignes)]
    for i, vi in enumerate(v):
        for j, vij in enumerate(vi):
            if vij != -1:
                cases = [ tuple(map(sum, zip((i,j), coord))) for coord in actions.values()]
                scores = [ vij if case_possible(c) else None for c in cases ]
                politique[i][j] = actions.keys()[np.argmax(scores)]
    return politique

print(g)
v = pl(g)
print(v)
p = get_politique(v)
print(p)
