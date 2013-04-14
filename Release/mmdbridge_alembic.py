import mmdbridge
from mmdbridge import *
import os
import math
from math import *
import time

# settings
start_frame = 5
end_frame = 500
export_normals = False
export_uvs = True
export_mode = 1 # 0 = create buffer every marerials, 1 = create buffer every objects


def export_mtl(mtlpath):
	if os.path.isfile(mtlpath):
		os.remove(mtlpath)

	mtlfile = open(mtlpath, 'a')

	for buf in range(get_vertex_buffer_size()):
		for mat in range(get_material_size(buf)):
			material_name = "material_" + str(buf) + "_" + str(mat)
			mtlfile.write("newmtl "+material_name+"\n")

			ambient = get_ambient(buf, mat)
			diffuse = get_diffuse(buf, mat)
			specular = get_specular(buf, mat)
			emissive = get_emissive(buf, mat)
			power = get_power(buf, mat)
			texture = get_texture(buf, mat)
			if len(texture) == 0:
				texture = get_exported_texture(buf, mat) + ".png"

			mtlfile.write("Ka "+str(ambient[0])+" "+str(ambient[1])+" "+str(ambient[2])+"\n")
			mtlfile.write("Kd "+str(diffuse[0])+" "+str(diffuse[1])+" "+str(diffuse[2])+"\n")
			mtlfile.write("Ks "+str(specular[0])+" "+str(specular[1])+" "+str(specular[2])+"\n")
			if (diffuse[3] < 1):
				mtlfile.write("d "+str(diffuse[3])+"\n")				
			mtlfile.write("Ns "+str(power)+"\n")
			#mtlfile.write("Ni 1.33\n")
			# lum = 1 no specular highlights, lum = 2 light normaly
			mtlfile.write("lum 1\n")
			if len(texture) > 0:
				mtlfile.write("map_Kd "+texture+"\n")
				if (diffuse[3] < 1):
					texname, ext = os.path.splitext(texture)
					if (ext is not "bmp") and (ext is not "png") and (ext is not "tif") and \
							(ext is not "BMP") and (ext is not "PNG") and (ext is not "TIF"):
						export_path = get_base_path() + "out\\" + texname + ".png"
						export_texture(buf, mat, export_path)
						mtlfile.write("map_d "+texname + ".png"+"\n")
					else:
						mtlfile.write("map_d "+texture+"\n")

outpath = get_base_path().replace("\\", "/") + "out/"
mtlpath = outpath + "alembic_file.mtl"
texture_export_dir = outpath

framenumber = get_frame_number()
if (framenumber == start_frame):
	export_mtl(mtlpath)
	copy_textures(texture_export_dir.replace("/", "\\"))
	messagebox("alembic export started")
	start_alembic_export("", export_mode, export_normals, export_uvs)

if (framenumber >= start_frame or framenumber <= end_frame):
	execute_alembic_export(framenumber)

if (framenumber >= end_frame):
	messagebox("alembic export ended at " + str(framenumber))
	end_alembic_export()