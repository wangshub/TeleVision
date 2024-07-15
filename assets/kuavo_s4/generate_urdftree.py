from ikpy.urdf.utils import get_urdf_tree

dot, urdf_tree = get_urdf_tree('urdf/biped_s4.urdf', out_image_path='biped_s4', root_element='torso')