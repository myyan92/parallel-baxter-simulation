from subprocess import call
import os, shutil

if os.path.isdir('new_graphinfo'):
    shutil.rmtree('new_graphinfo')
os.mkdir('new_graphinfo')

f = open('new_graphinfo/topics.txt','w')
call(['rostopic', 'list'], stdout=f)
f = open('new_graphinfo/nodes.txt','w')
call(['rosnode', 'list'], stdout=f)

with open('new_graphinfo/topics.txt') as f:
    topics = f.readlines()
topics = [t.strip() for t in topics]
for i,t in enumerate(topics):
    f = open('new_graphinfo/topic_%d.txt'%(i+1),'w')
    call(['rostopic', 'info', t], stdout=f)

with open('new_graphinfo/nodes.txt') as f:
    nodes = f.readlines()
nodes = [n.strip() for n in nodes]
for i,n in enumerate(nodes):
    f = open('new_graphinfo/node_%d.txt'%(i+1),'w')
    call(['rosnode', 'info', n], stdout=f)


