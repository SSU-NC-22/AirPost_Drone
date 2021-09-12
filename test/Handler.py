import json

class Handler():
	def __init__(self):
		print("handler init")

	def run(self, msg):
		print("handler run")
		try: 
			sub_str = str(msg.decode("utf-8", "ignore")) #string data
			print("subscribe: ", type(sub_str), sub_str)
			sub_dict = json.loads(sub_str) # dict data
			print(sub_dict)
		except:
				print("handler error")