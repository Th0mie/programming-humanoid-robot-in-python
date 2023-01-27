from xmlrpc.server import SimpleXMLRPCServer

class MyFuncs:
	def plus(self, x, y):
		return x+y

	def minus(self, x, y):
		return x-y

server = SimpleXMLRPCServer(('localhost', 8000))
print(server)
server.register_instance(MyFuncs())

server.serve_forever()