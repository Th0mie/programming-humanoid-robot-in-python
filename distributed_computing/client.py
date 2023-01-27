import xmlrpc.client

client = xmlrpc.client.ServerProxy('http://localhost:8000')
print(client.plus(2,3))
print(client.minus(5,1))