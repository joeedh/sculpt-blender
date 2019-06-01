class Console {
    format(obj, tlevel=0) {
        let tab = "";
        for (let i = 0; i < tlevel; i++) {
            tab += "  ";
        }

        if (typeof obj == "function") {
            return "[function " + obj.name + "]";
        } else if (typeof obj != "object") {
            return "" + obj;
        } else if (obj instanceof Array) {
            let ret = "[";
            for (let i = 0; i < obj.length; i++) {
                if (i > 0) {
                    ret += ", ";
                }
                ret += this.format(obj[i], tlevel + 1);
            }
            ret += "]";

            return ret;
        } else {
            let ret = "{\n";

            for (let k of Object.getOwnPropertyNames(obj)) {
                ret += tab + "  " + k + " : " + this.format(obj[k], tlevel+1) + ",\n";
            }
            for (let k of Object.getOwnPropertySymbols(obj)) {
                ret += tab + "  [" + k.description + "] : " + this.format(obj[k], tlevel + 1) + ",\n";
            }
            ret += tab + "}\n";
            return ret;
        }
    }

    [Symbol.iterator]() {
        return undefined;
    }

    log() {
        let line = "";

        for (let i = 0; i < arguments.length; i++) {
            line += this.format(arguments[i]) + " ";
        }
        line += "\n";

        _native_print(line);
    }

    warn() {
        this.log.apply(this, arguments);
    }

    trace() {
        this.log.apply(this, arguments);
    }
}

let console = new Console();
