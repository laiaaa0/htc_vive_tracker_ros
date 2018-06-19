import json

def decode_json (filename):
	my_file = open (filename,'r')
	my_object = json.load (my_file)
	return my_object

def encode_xml (filename,obj):

	f = open (filename,'w')
	f.write("<rosparam>\n")
	for key in obj.keys():
		current_value = obj[key]
		if (type(current_value)==dict):
			for k in current_value.keys():
				f.write(str(k) + ": "+str(current_value[k])+"\n")
		else:
			f.write (str(key)+": "+str (current_value)+"\n")
	f.write("</rosparam>\n")

if __name__=="__main__":
	o = decode_json("calibration.json")
	encode_xml ("calibration.xml",o)
