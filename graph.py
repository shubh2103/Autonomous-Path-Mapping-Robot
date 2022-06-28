n=13
m=26
graph={}
def createGraph():
    for i in range(n):
        for j in range(m):
            v=i*n+j
            graph[v]=[]
            if initmatrix[i][j]==-1:
                continue
            if i>0 and i<12 and j>0 and j<25:
                if initmatrix[i-1][j]!=-1:
                    graph[v].append((i-1)*n+j)
                if initmatrix[i+1][j]!=-1:
                    graph[v].append((i+1)*n+j)
                if initmatrix[i][j-1]!=-1:
                    graph[v].append(i*n+j-1)
                if initmatrix[i][j+1]!=-1:
                    graph[v].append(i*n+j+1)
            if i==0:
                if initmatrix[i+1][j]!=-1:
                    graph[v].append((i+1)*n+j)
                if initmatrix[i][j-1]!=-1:
                    graph[v].append(i*n+j-1)
                if initmatrix[i][j+1]!=-1:
                    graph[v].append(i*n+j+1)
            if i==12:
                if initmatrix[i-1][j]!=-1:
                    graph[v].append((i-1)*n+j)
                if initmatrix[i][j-1]!=-1:
                    graph[v].append(i*n+j-1)
                if initmatrix[i][j+1]!=-1:
                    graph[v].append(i*n+j+1)
            if j==0:
                if initmatrix[i-1][j]!=-1:
                    graph[v].append((i-1)*n+j)
                if initmatrix[i+1][j]!=-1:
                    graph[v].append((i+1)*n+j)
                if initmatrix[i][j+1]!=-1:
                    graph[v].append(i*n+j+1)
            if j==25:
                if initmatrix[i-1][j]!=-1:
                    graph[v].append((i-1)*n+j)
                if initmatrix[i+1][j]!=-1:
                    graph[v].append((i+1)*n+j)
                if initmatrix[i][j-1]!=-1:
                    graph[v].append(i*n+j-1)