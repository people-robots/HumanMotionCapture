import json
import os
import random

def meta_counter(name,filename='./metadata.json'):
	if name is None:
		return
	data = []
	bit = 'r+'
	with open(filename, bit) as f:
		data = json.load(f)
		if(name in data[0].keys()):
			data[0][name] += 1
		else:
			data[0][name] = 1
		f.seek(0)
		json.dump(data, f, indent=4, sort_keys=True)
		f.truncate()

def select_random(max=4, filename='./metadata.json'):
	with open(filename, 'r') as f:
		data = json.load(f)
		print (data)
		values = [k for d in data for k,v in d.items() if v < max]
		try:
			x = random.choice(values)
			print(x)
			return x
		except IndexError:
			print("Nothing Found")
