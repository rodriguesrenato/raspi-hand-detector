import json

class Configs:
	def __init__(self, file):
		self.file = file;
		self.data = None;
		self.load();

	def load(self):
		with open(self.file, 'r') as f:
			self.data = json.load(f);

	def save(self):
		with open(self.file, 'w') as f:
			json.dump(self.data, f);

	def set(self, key, value):
		self.data[key] = value;
		self.save();

	def get(self, key):
		return self.data[key];
