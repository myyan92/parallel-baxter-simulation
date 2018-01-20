def parseNodeInfo(f):
    lines = f.readlines()
    nodename=''
    pub=[]
    sub=[]
    srv=[]
    mode = None
    for l in lines:
        if l.startswith('Node'):
            nodename=l.strip().split()[1][1:-1]
        if l.startswith('Publications'):
            mode='pub'
        if l.startswith('Subscriptions'):
            mode='sub'
        if l.startswith('Services'):
            mode='srv'
        if l.startswith('contacting'):
            break
        if l.startswith(' * '):
            topic = l.strip().split()[1]
            if mode=='pub': pub.append(topic)
            if mode=='sub': sub.append(topic)
            if mode=='srv': srv.append(topic)
    return nodename, pub, sub, srv

with open('reference_graphinfo/nodes.txt') as f:
    lines = f.readlines()
ref_nodes = [l.strip() for l in lines]
with open('new_graphinfo/nodes.txt') as f:
    lines = f.readlines()
new_nodes = [l.strip() for l in lines]
nodemap = {}
with open('reference_graphinfo/topics.txt') as f:
    lines = f.readlines()
ref_topics = [l.strip() for l in lines]
with open('new_graphinfo/topics.txt') as f:
    lines = f.readlines()
new_topics = [l.strip() for l in lines]
topicmap = {}

for n in ref_nodes:
    nodemap[n]=[]
for n in new_nodes:
    print("matching node %s" % (n))
    matching_n = n.replace('/alice', '').replace('/bob', '')
    matched = False
    for rn in ref_nodes:
        if rn.endswith(matching_n):
            assert(not matched)
            nodemap[rn].append(n)
            print("matched to node %s" %(rn))
            matched = True
    if not matched:
        print("WARNING: node %s not matched to ref node" % (n))
for n in nodemap:
    if not nodemap[n]:
        print("WARNING: node %s not matched to new node" % (n))

for n in ref_topics:
    topicmap[n]=[]
for n in new_topics:
    print("matching topic %s" % (n))
    matching_n = n.replace('/alice', '').replace('/bob', '')
    matched = False
    for rn in ref_topics:
        if rn == matching_n or rn.replace('/robot','') == matching_n or rn.replace('/cameras','',1) == matching_n:
            assert(not matched)
            topicmap[rn].append(n)
            print("matched to topic %s" %(rn))
            matched = True
    if not matched:
        print("WARNING: topic %s not matched to ref topic" % (n))
for n in topicmap:
    if not topicmap[n]:
        print("WARNING: topic %s not matched to new topic" % (n))

for rn in nodemap:
    for n in nodemap[rn]:
        # compare node info
        print("comparing node info %s and %s" % (rn,n))
        idxr=ref_nodes.index(rn)
        with open('reference_graphinfo/node_%d.txt' % (idxr+1)) as f:
            nodenamer, pubr, subr, srvr = parseNodeInfo(f)
            assert(nodenamer==rn)
        idx =new_nodes.index(n)
        with open('new_graphinfo/node_%d.txt' % (idx+1)) as f:
            nodename, pub, sub, srv = parseNodeInfo(f)
            assert(nodename==n)
        for t in pubr:
            matching_t = topicmap[t][0]
            if (not '/alice' in matching_t) or (not '/bob' in n):
                if not matching_t in pub:
                    print('WARNING: ref node topic %s not found in new node' % (t))
                else:
                    pub.remove(matching_t)
            if (not '/alice' in n) and len(topicmap[t])>1:
                matching_t = topicmap[t][1]
                if not matching_t in pub:
                    print('WARNING: ref node topic %s not found in new node' % (t))
                else:
                    pub.remove(matching_t)
        if pub:
            print('WARNING: some topics in new node not matched to ref node')
            print(pub)
        for t in subr:
            matching_t = topicmap[t][0]
            if (not '/alice' in matching_t) or (not '/bob' in n):
                if not matching_t in sub:
                    print('WARNING: ref node topic %s not found in new node' % (t))
                else:
                    sub.remove(matching_t)
            if (not '/alice' in n) and len(topicmap[t])>1:
                matching_t = topicmap[t][1]
                if not matching_t in sub:
                    print('WARNING: ref node topic %s not found in new node' % (t))
                else:
                    sub.remove(matching_t)
        if sub:
            print('WARNING: some topics in new node not matched to ref node')
            print(sub)

