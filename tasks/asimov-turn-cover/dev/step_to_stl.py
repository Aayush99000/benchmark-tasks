"""STEP to STL (tessellation 0.3 mm) with OpenCascade (pip install cadquery-ocp). Usage: python dev/step_to_stl.py in.STEP out.stl"""
import sys
from OCP.STEPControl import STEPControl_Reader
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
r = STEPControl_Reader(); r.ReadFile(sys.argv[1]); r.TransferRoots(); s = r.OneShape()
BRepMesh_IncrementalMesh(s, 0.3, False, 0.3, True)
StlAPI_Writer().Write(s, sys.argv[2])
