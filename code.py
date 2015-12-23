
# coding: utf-8

# In[26]:

get_ipython().magic('pylab')


# In[27]:

nblignes=10
nbcolonnes=10

g = np.zeros((nblignes,nbcolonnes), dtype=numpy.int)

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
        elif z < pmur+ pblanc: c=0
        elif z < pmur+ pblanc + pverte: c=1
        elif z < pmur+ pblanc +pverte + pbleue: c=2
        elif z< pmur + pblanc + pverte + pbleue +prouge: c=3
        else: c=4
        g[i,j]=c
g[0,0]=0
g[0,1]=0
g[2,0]=0
g[nblignes-1,nbcolonnes-1]=0
g[nblignes-2,nbcolonnes-1]=0
g[nblignes-1,nbcolonnes-2]=0


# In[28]:

g


# In[70]:

gamma = 0.9

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

actions = {'R': (-2, -1),
           'T': (-1, -2),
           'Y': (1, -2),
           'U': (2, -1),
           'F': (2, 1),
           'G': (1, 2),
           'H': (-1, 2),
           'J': (-2, 1)}

def case_possible(case):
    if not 0 <= case[0] < nblignes or not 0 <= case[1] < nbcolonnes or g[case] == -1:
        return False
    return True

def get_possible_destinations(case):
    cases = [ tuple(map(sum, zip(case, coord))) for coord in actions.values()]
    return [ c for c in cases if case_possible(c)]

vt = np.zeros((nblignes,nbcolonnes))
vt[-1][-1]=1000
print(vt)
vtplusun = 0
i = 0
while True:
    i += 1
    if i % 20 == 0: print(np.array(vt))
    vtplusun = [[max([cost(dest)+gamma*vt[dest[0]][dest[1]] for dest in get_possible_destinations((i,j))]) for i in range(nblignes)] for j in range(nbcolonnes)]
    if np.array_equal(vt, vtplusun):
        break
    vt=vtplusun


# In[62]:

vt[1][2]


# In[39]:

get_possible_destinations((5,5))


# In[31]:

v = [np.zeros((nblignes,nbcolonnes))]


# In[32]:

for _,j in actions.items():
    print (j)


# In[33]:

v2 = [[max([cost(dest) for dest in get_possible_destinations((i,j))]) for i in range(nblignes)] for j in range(nbcolonnes)]


# In[34]:

g


# In[71]:

plot(ear)


# In[ ]:
