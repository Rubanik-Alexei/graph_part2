import bs4
import os
import io
import sys
import svgwrite
import csv
import math as m
import heapq
import time

class PriorityQueue:
	def __init__(self):
		self.elements = []

	def empty(self):
		return len(self.elements) == 0

	def put(self, item, priority):
		return heapq.heappush(self.elements, (priority, item))

	def get(self):
		return heapq.heappop(self.elements)[1]


def heuristic(Firstnode, SecondNode, nodesCor):
	#return float(m.sqrt(m.pow(float(nodesCor[Firstnode][0]) - float(nodesCor[SecondNode][0]), 2) + m.pow(float(nodesCor[Firstnode][1]) - float(nodesCor[SecondNode][1]), 2)))
	return abs(float(nodesCor[Firstnode][0]) - float(nodesCor[SecondNode][0])) + abs(float(nodesCor[Firstnode][1]) - float(nodesCor[SecondNode][1]))
	# tmp1 = abs(float(nodesCor[Firstnode][0]) - float(nodesCor[SecondNode][0]))
	# tmp2 = abs(float(nodesCor[Firstnode][1]) - float(nodesCor[SecondNode][1]))
	# if tmp1 > tmp2:
	# 	return tmp1
	# else:
	# 	return tmp2



def astar(adjlist, start, DestinationPoint, nodesCor,):
	frontier = PriorityQueue()
	frontier.put(start, 0)
	came_from = {}
	cost_so_far = {}
	came_from[start] = None
	cost_so_far[start] = 0

	while not frontier.empty():
		current = frontier.get()

		if current == DestinationPoint:
			break

		for _next in adjlist[current]:
			new_cost = cost_so_far[current] + adjlist[current][0][1] #+ cost.get(current.__str__() + ', ' + next.__str__())
			if _next[0] not in cost_so_far or new_cost < cost_so_far[_next[0]]:
				cost_so_far[_next[0]] = new_cost
				priority = new_cost + heuristic(DestinationPoint, _next[0], nodesCor)
				frontier.put(_next[0], priority)
				came_from[_next[0]] = current

	return came_from, cost_so_far

mult=float(sys.argv[2])
#mult = 10000

def NearestNode(refKey,array,nodesCor,prevdist,pointRef,custom=False):
	points=[]
	for point in array:
		if not custom:
			point = nodesCor[point]
		else: pass
		points.append(m.sqrt(m.pow(float(nodesCor[refKey][0]) - float(point[0]),2) + m.pow(float(nodesCor[refKey][1]) - float(point[1]),2)))
		if(prevdist[points.index(min(points))]>min(points)):
			prevdist[points.index(min(points))]=min(points)
			pointRef[points.index(min(points))] = refKey
	return pointRef,prevdist



def Metric(NodesArray, nodesCor):
	sumdist = 0
	for i in range (len(NodesArray)-1):
		 sumdist += float(m.sqrt(m.pow(float(nodesCor[NodesArray[i]][0])-float(nodesCor[NodesArray[i+1]][0]),2)+m.pow(float(nodesCor[NodesArray[i]][1])-float(nodesCor[NodesArray[i+1]][1]),2)))
	return sumdist



def Levit(n, start, adjlist, nodeindexes, indexnodes):
	from collections import deque
	q = deque()
	INF=m.inf
	dist = [INF] * n
	dist[start] = 0
	prev = [None] * n
	q.append(start)
	id=[2]*n
	id[start] = 1
	print(len(q))
	while len(q) > 0:
		v = q.popleft()
		#print('V' + str(v))
		id[v] = 0
		ref = indexnodes[v]
		for j in adjlist[ref]:
			to = nodeindexes[j[0]]
			#print(id[to])
			if dist[to] > dist[v] + adjlist[ref][0][1]:
				dist[to] = dist[v] + adjlist[ref][0][1]
				if id[to] == 2:
					#print('2')
					q.append(to)
				elif id[to] == 0:
					#print('0')
					q.appendleft(to)
				prev[to] = v
				id[to] = 1
		#print("++")
	return dist, prev


def Dijkstra(n, start, adjlist, nodeindexes, indexnodes):
	INF=m.inf
	dist = [INF] * n
	dist[start] = 0
	prev = [None] * n
	used = [False] * n
	min_vertex = start
	min_dist = 0
	while min_dist < INF:
		i = min_vertex
		#print(i)
		used[i] = True
		ref = indexnodes[i]
		#print (ref)
		for j in adjlist[ref]:
			t = nodeindexes[j[0]]
			#print(dist[i])
			#print(adjlist[ref][0][1])
			#print(dist[t])
			if dist[i] + adjlist[ref][0][1] < dist[t]:
				dist[t] = dist[i] + adjlist[ref][0][1]
				prev[t] = i
		min_dist = INF
		for i in range(n):
			if not used[i] and dist[i] < min_dist:
				min_dist = dist[i]
				min_vertex = i

	return dist, prev



def drowLineFromTo(PathNodes,maxPoint,minPoint,nodesCor,mult,Color='red'):
	begPoint=[0,0]
	for node in PathNodes:
		if begPoint==[0,0]:
			begPoint=[int((float(nodesCor[node][0])-minPoint[0])*mult),int((maxPoint[1]-float(nodesCor[node][1]))*mult)]
		else:
			endPoint=[int((float(nodesCor[node][0])-minPoint[0])*mult),int((maxPoint[1]-float(nodesCor[node][1]))*mult)]
			dwg.add(dwg.line((begPoint[0], begPoint[1]), (endPoint[0], endPoint[1]), stroke=Color))
			begPoint=endPoint

def drowPoint(PathNodes,maxPoint,minPoint,nodesCor,mult,Color='red',size=2):
	for node in PathNodes:
		Point=[int((float(nodesCor[node][0])-minPoint[0])*mult),int((maxPoint[1]-float(nodesCor[node][1]))*mult)]
		dwg.add(dwg.circle(center=(Point[0], Point[1]), r=size, stroke=Color))

def MakeAdjList(highways, nodesCor):
	adjacencyList = {}
	for key in highways.keys():
		for ref in highways[key]:
			adjacencyList[ref]=[]

	for key in highways.keys():
		counter=0
		for ref in highways[key]:
			if counter == len(highways[key])-1:
				adjacencyList[ref].append([highways[key][counter-1], Metric([ref,highways[key][counter-1]],nodesCor)])
			else:
				if counter == 0:
					adjacencyList[ref].append([highways[key][counter+1],Metric([ref,highways[key][counter+1]],nodesCor)])
				else:
					adjacencyList[ref].append([highways[key][counter-1],Metric([ref,highways[key][counter-1]],nodesCor)])
					adjacencyList[ref].append([highways[key][counter+1],Metric([ref,highways[key][counter+1]],nodesCor)])
			counter+=1

	#print('Make adjacencyList .... OK')
	return adjacencyList






highways={}
nodesCor={}
Targets=[]
TargetMinDist=[]
TargetRefs=[]
CustomMinDist=[]
CustomRefs=[]

dwg = svgwrite.Drawing('graph.svg', profile='tiny')
map_file = sys.argv[1]
#map_file = 'tula.osm'

soup=bs4.BeautifulSoup(io.open(map_file,encoding='utf-8'),'xml')
ways=soup.findAll('way')
nodes=soup.findAll('node')

#CustomPoints=[[37.4,54.2]]
CustomPoints=[[sys.argv[3], sys.argv[4]]]


# steplat = 0.010815431
# steplon = 0.009176614
# CustomPoints = []
# startlat = 37.0911631
# startlon = 54
# c_er1 = 0
# c_er2 = 0
# while c_er1 < 10:
# 	tmp = startlon
# 	while c_er2 < 10:
# 		CustomPoints.append([startlat, tmp])
# 		tmp += steplon
# 		c_er2 += 1
# 	c_er2 = 0
# 	startlat += steplat
# 	c_er1 += 1
# print(CustomPoints)


for node in nodes:
	nodesCor[node.attrs['id']]=[node.attrs['lon'],node.attrs['lat']]

for i in CustomPoints:
	CustomRefs.append('')
	CustomMinDist.append(10)

for node in nodes:
		tag = node.findAll('tag')
		for t in tag:
			if t.attrs['k'] == 'amenity' and t.attrs['v'] == 'hospital':
				Targets.append(node.attrs['id'])
				TargetMinDist.append(10)
				TargetRefs.append('')
Targets.append('2751579168')
TargetMinDist.append(10)
TargetRefs.append('')

v=['motorway', 'trunk', 'primary', 'secondary', 'tertiary', 'unclassified', 'residential', 'service', 'road']
for way in ways:
	tag = way.find('tag',k='highway',v=v)
	if tag!=None:
		nd=way.findAll('nd')
		ref=[]
		for n in nd:
			ref.append(n.attrs['ref'])
		highways[way.attrs['id']]=ref
print('Make highways .... OK')

maxPoint=[0,0]
minPoint=[90,180]

for nodeKeys in nodesCor.keys():
	if maxPoint[0]<float(nodesCor[nodeKeys][0]):
		maxPoint[0]=float(nodesCor[nodeKeys][0])
	if maxPoint[1]<float(nodesCor[nodeKeys][1]):
		maxPoint[1]=float(nodesCor[nodeKeys][1])
	if minPoint[0]>float(nodesCor[nodeKeys][0]):
		minPoint[0]=float(nodesCor[nodeKeys][0])
	if minPoint[1]>float(nodesCor[nodeKeys][1]):
		minPoint[1]=float(nodesCor[nodeKeys][1])

print('Max:' + str(maxPoint[0]) + ' ' + str(maxPoint[1])  )
print('Min:' + str(minPoint[0]) + ' ' + str(minPoint[1])  )

for key in highways.keys():
	begPoint=[0,0]
	for refKey in highways[key]:
		TargetRefs, TargetMinDist = NearestNode(refKey,Targets,nodesCor,TargetMinDist,TargetRefs)
		CustomRefs ,CustomMinDist = NearestNode(refKey,CustomPoints,nodesCor,CustomMinDist,CustomRefs,custom=True)

		if begPoint==[0,0]:
			begPoint=[int((float(nodesCor[refKey][0])-minPoint[0])*mult),int((maxPoint[1]-float(nodesCor[refKey][1]))*mult)]
			dwg.add(dwg.circle(center=(begPoint[0], begPoint[1]), r=0, stroke='red'))
		else:
			endPoint=[int((float(nodesCor[refKey][0])-minPoint[0])*mult),int((maxPoint[1]-float(nodesCor[refKey][1]))*mult)]
			dwg.add(dwg.circle(center=(endPoint[0], endPoint[1]), r=0, stroke='red'))
			dwg.add(dwg.line((begPoint[0], begPoint[1]), (endPoint[0], endPoint[1]), stroke='black'))
			#print(begPoint,endPoint,maxPoint,minPoint)
			begPoint=endPoint	
print('Draw Map .... OK')
print(CustomRefs)

for Target in Targets:
	point = [int((float(nodesCor[Target][0])-minPoint[0])*mult),int((maxPoint[1]-float(nodesCor[Target][1]))*mult)]
	dwg.add(dwg.circle(center=(point[0],point[1]), r=2, stroke='blue'))
print('Draw Targets .... OK')	

for CustomPoint in CustomPoints:
	point = [int((float(CustomPoint[0])-minPoint[0])*mult),int((maxPoint[1]-float(CustomPoint[1]))*mult)]
	dwg.add(dwg.circle(center=(point[0],point[1]), r=4, stroke='red'))
print('Draw CustomPoints .... OK')	

for CustomRef in CustomRefs:
	point = [int((float(nodesCor[CustomRef][0])-minPoint[0])*mult),int((maxPoint[1]-float(nodesCor[CustomRef][1]))*mult)]
	dwg.add(dwg.circle(center=(point[0],point[1]), r=4, stroke='green'))
print('Draw Nearest CustomPoints on map.... OK')	

colors = ['red','green','brown','pink','yellow','khaki','purple','blue','orange','aqua']
c = 0
adjlist=MakeAdjList(highways, nodesCor)
nodeindexes = {}
indexnodes = {}

#print(adjlist)
i = 0
j = 0
for key in adjlist.keys():
	nodeindexes.update({key:i})
	i+=1
for key in adjlist.keys():
	indexnodes.update({j:key})
	j+=1

paths_aStar=[]
costs=[]
cntr = 0
timing = []
print(str(len(CustomRefs)))
for CustomRef in CustomRefs:
	print(CustomRef)
	print(len(adjlist))
	nodeind = nodeindexes.get(CustomRef)
	print(nodeind)
	d_start = time.time()
	dist, prev= Levit(len(adjlist), nodeind , adjlist, nodeindexes, indexnodes)
	d_end = time.time()
	dtime = d_end - d_start
	print(str(dtime))
	print('NewIteration')
	timing.append((dtime))
	f = open('paths.csv', 'w', newline='', encoding='utf-8')
	awriter = csv.DictWriter(f, fieldnames=['node', 'path'])
	# tmp = 0
	for TargetRef in TargetRefs:

		point = [int((float(nodesCor[TargetRef][0])-minPoint[0])*mult),int((maxPoint[1]-float(nodesCor[TargetRef][1]))*mult)]
		dwg.add(dwg.circle(center=(point[0],point[1]), r=2, stroke='pink'))
		# a_start = time.time()
		# pathstar, cost = astar(adjlist, CustomRef, TargetRef, nodesCor)
		# a_end = time.time()
		# a_time = a_end - a_start
		# tmp += a_time
	# 	#paths_aStar.append(pathstar)
	# 	#costs.append(cost)
		paths = []
		j = nodeindexes.get(TargetRef)
		path=[]
		while j is not None:
			path.append(indexnodes.get(j))
			j = prev[j]
		path = path[::-1]
		paths.append(path)
		awriter.writerow({'node': TargetRef, 'path': path})
		#print(path)
		drowLineFromTo(path,maxPoint,minPoint,nodesCor,mult,Color=colors[c])
		route=Metric(path,nodesCor)
	# 	print('Route is ... ',route)
		c +=1
	# 	print('NewIteration')
	# # #f = open('pathsastar.csv', 'w', newline='', encoding='utf-8')
	# # #awriter = csv.DictWriter(f, fieldnames=['node', 'path'])
	# # for path in paths_aStar:
	# # 	key = path[TargetRefs[cntr]]
	# # 	pathcsv=[]
	# # 	while not key == None:
	# # 		_id = adjlist[key][0][0]
	# # 		pathcsv.append(_id)
	# # 		key=path[key]
	# # 	pathcsv.reverse()
	# # 	awriter.writerow({'node': TargetRefs[cntr], 'path': pathcsv})
	# # 	cntr+=1
	f.close()

	# print('NewIteration')
	#timing.append((tmp))
print('Draw path from CustomPoints to Targets.... OK')
sumtimes = 0
for t in timing:
	sumtimes += t
average = sumtimes/len(timing)
print('Average:' + str(average))
dpoint=[(maxPoint[0]-minPoint[0]),maxPoint[1]-minPoint[1]]
dwg.viewbox(0, 0, int((maxPoint[0]-minPoint[0])*mult), int((maxPoint[1]-minPoint[1])*mult))
dwg.save()



