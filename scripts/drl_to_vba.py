import re
import sys

in_file = open(sys.argv[1], 'r')
out_file = open(sys.argv[2], 'w')
for line in in_file.readlines():
  if re.match("X.+Y.+", line):
    x, y = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", line)
    x = float(x)
    y = float(y)
    x = x * 0.0254
    y = y * 0.0254
    out_file.write("Set skPoint = Part.SketchManager.CreatePoint({:f}, {:f}, 0#)\n".format(x,y))
