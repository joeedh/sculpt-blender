file = open("jscode.js", "r");
buf = file.read()
file.close()

lines = buf.split("\n");
out = "char jscode[] = {\n"
totchar = 0

for l in lines:
    l += "\n";

    for c in l:
        c = ord(c)
        chunk = hex(c) + ","
        out += chunk
        totchar += len(chunk)

        if totchar > 74:
            out += "\n"
            totchar = 0

out += "0};\n"

file = open("_jscode_js.cpp", "w")
file.write(out)
file.close()
