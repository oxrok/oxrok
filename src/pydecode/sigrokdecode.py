# The sigrokdecode module for the RustPython backend: what the decoders
# import, the Decoder base class and the output constants. The sample
# stream and the outputs are in the native _oxrok module.

import _oxrok
import builtins
import math

OUTPUT_ANN = 0
OUTPUT_PYTHON = 1
OUTPUT_BINARY = 2
OUTPUT_META = 3
SRD_CONF_SAMPLERATE = 10000

_CONDS = {
    'l': 0, 'low': 0,
    'h': 1, 'high': 1,
    'r': 2, 'rising': 2,
    'f': 3, 'falling': 3,
    'e': 4, 'edge': 4,
    's': 5, 'stable': 5,
}


def _sum(iterable, /, start=0):
    # sum() as CPython 3.12 and newer have it: floats are added with
    # Neumaier's compensation, which RustPython's sum() lacks, so
    # averages like the timing decoder's differ in the last digit.
    for kind, name in ((str, 'strings'), (bytes, 'bytes'),
                       (bytearray, 'bytearray')):
        if isinstance(start, kind):
            join = "''" if kind is str else "b''"
            raise TypeError("sum() can't sum %s [use %s.join(seq) instead]"
                            % (name, join))
    it = iter(iterable)
    result = start
    if type(result) is int:
        for item in it:
            result = result + item
            if type(result) is not int:
                break
        else:
            return result
    if type(result) is float:
        f, c = result, 0.0
        for item in it:
            if type(item) is float:
                t = f + item
                if abs(f) >= abs(item):
                    c += (f - t) + item
                else:
                    c += (item - t) + f
                f = t
                continue
            if isinstance(item, int) and -2 ** 63 <= item < 2 ** 63:
                f += float(item)
                continue
            if c and math.isfinite(c):
                f += c
            result = f + item
            break
        else:
            if c and math.isfinite(c):
                f += c
            return f
    for item in it:
        result = result + item
    return result


builtins.sum = _sum


class Decoder:
    # Decoders need not call this __init__, so every attribute set here
    # is also looked up with a default.

    def register(self, output_type, proto_id=None, meta=None):
        outputs = self.__dict__.setdefault('_oxrok_outputs', [])
        name = meta[1] if meta is not None else None
        outputs.append((output_type, name))
        return len(outputs) - 1

    def has_channel(self, channel):
        channels = self.__dict__.get('_oxrok_channels', ())
        return 0 <= channel < len(channels) and channels[channel]

    def put(self, ss, es, output_id, data):
        outputs = self.__dict__.get('_oxrok_outputs', [])
        if not 0 <= output_id < len(outputs):
            raise ValueError('unknown output id')
        kind, name = outputs[output_id]
        ss, es = max(ss, 0), max(es, 0)
        if kind == OUTPUT_ANN:
            _oxrok.put_ann(ss, es, data[0], list(data[1]))
        elif kind == OUTPUT_BINARY:
            _oxrok.put_binary(ss, es, data[0], bytes(data[1]))
        elif kind == OUTPUT_META:
            _oxrok.put_meta(ss, es, name or '', data)
        else:
            _oxrok.put_python(ss, es, data)

    def wait(self, conds=None):
        # samplenum and matched are set on the instance, as
        # libsigrokdecode does, so class attributes of the same names
        # are not in the way.
        if conds is None:
            groups = []
        elif isinstance(conds, dict):
            groups = [conds]
        else:
            groups = list(conds)
        spec = []
        for group in groups:
            terms = []
            for key, value in group.items():
                if isinstance(key, str):
                    if key == 'skip':
                        terms += (-1, max(int(value), 0))
                    continue
                if value not in _CONDS:
                    raise ValueError("unknown condition '%s'" % value)
                terms += (int(key), _CONDS[value])
            spec.append(terms)
        pins, self.samplenum, self.matched = _oxrok.wait(spec)
        return pins


def _attr(cls, name, default):
    value = getattr(cls, name, None)
    return default if value is None else value


def _option_value(v):
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, (int, float)):
        return v
    return str(v)


def _oxrok_info(module_name):
    # The class attributes of a decoder as plain lists, strings and
    # numbers, in the order the host reads them.
    module = __import__(module_name)
    cls = module.Decoder
    if cls.api_version != 3:
        raise ValueError('api_version %s is not 3' % cls.api_version)

    def channels(name):
        return [(c['id'], c['name'], c.get('desc', ''))
                for c in _attr(cls, name, ())]

    options = []
    for o in _attr(cls, 'options', ()):
        values = [_option_value(v) for v in o.get('values', ())]
        options.append((o['id'], o['desc'], _option_value(o['default']), values))
    return [
        _attr(cls, 'id', ''),
        _attr(cls, 'name', ''),
        _attr(cls, 'longname', ''),
        _attr(cls, 'desc', ''),
        _attr(cls, 'license', ''),
        list(_attr(cls, 'inputs', ())),
        list(_attr(cls, 'outputs', ())),
        list(_attr(cls, 'tags', ())),
        channels('channels'),
        channels('optional_channels'),
        options,
        [tuple(a) for a in _attr(cls, 'annotations', ())],
        [(r[0], r[1], list(r[2])) for r in _attr(cls, 'annotation_rows', ())],
        [tuple(b) for b in _attr(cls, 'binary', ())],
        (module.__doc__ or '').strip(),
    ]
