import sigrokdecode as srd

class KlineFsm:
    class State:
        Header = 'HEADER'
        Data = 'DATA'
        Checksum = 'CHECKSUM'
        Error = 'ERROR'

    def transit(self, target_state):
        if not self._transition_allowed(target_state):
            return False
        self.state = target_state
        return True

    def _transition_allowed(self, target_state):
        if target_state == KlineFsm.State.Error:
            return True
        return target_state in self.allowed_state[self.state]

    def reset(self):
        self.state = KlineFsm.State.Header

    def __init__(self):
        a = dict()
        a[KlineFsm.State.Header]         = (KlineFsm.State.Data,)
        a[KlineFsm.State.Data]         = (KlineFsm.State.Data, KlineFsm.State.Checksum)
        a[KlineFsm.State.Checksum]     = (KlineFsm.State.Header,)
        a[KlineFsm.State.Error]        = (KlineFsm.State.Header,)
        self.allowed_state = a

        self.state = None
        
        self.reset()

class Decoder(srd.Decoder):
    api_version = 3
    id = 'k-line'
    name = 'K-Line'
    longname = 'eee'
    desc = 'ee'
    license = 'gplv2+'
    inputs = ['uart']
    outputs = []
    tags = ['Automotive']
    options = ()
    annotations = (
        ('data', 'K-Line data'),
        ('control', 'Protocol info'),
        ('error', 'Error descriptions'),
        ('inline_error', 'Protocol violations and errors'),
    )
    annotation_rows = (
        ('data', 'Data', (0, 1, 3)),
        ('error', 'Error', (2,)),
    )
    binary = (
        ('tester', 'Tester requests'),
        ('device', 'Device responses'),
        ('dump', 'TSV formatted dump'),
    )


    def __init__(self):
        self.reset()

    def reset(self):
        self.fsm = KlineFsm()
        self.frame = []
        self.frame_len = 0
        self.ss_block = None
        self.es_block = None
        self.frame_start = None
        self.frame_end = None
        self.tsv_header = False

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_python = self.register(srd.OUTPUT_PYTHON)
        self.out_binary = self.register(srd.OUTPUT_BINARY)

    def putx(self, data):
        self.put(self.ss_block, self.es_block, self.out_ann, data)

    def handle_header(self, value):
        if value == 0x00 or value < 0x80:
            # Nothing to do
            self.putx([2, ['Start of frame not >=0x80', 'Not >=0x80', '<0x80']])
            return

        self.frame_len = value - 0x80
        self.frame_len += 1 + 1 + 1  # Include this header, + dest, + src
        self.fsm.transit(KlineFsm.State.Data)
        self.handle_data(value)

    def handle_data(self, value):
        self.frame.append(value)

        bytes_received = len(self.frame)
        if bytes_received == 1:
            self.putx([1, ['Header (len %d)' % (self.frame_len), 'Header', 'Hdr']])
        elif bytes_received == 2:
            self.frame_start = self.ss_block
            self.putx([1, ['Destination (%02X)' % (value), 'Destination', 'Dest']])
            self.destination_id = value
        elif bytes_received == 3:
            self.putx([1, ['Source (%02X)' % (value), 'Source', 'Src']])
            self.source_id = value
        else:
            self.putx([1, ['Data  (%02X)' % (value), 'Data', 'Data']])
        
        if bytes_received == self.frame_len:
            self.frame_end = self.es_block
            self.fsm.transit(KlineFsm.State.Checksum)

    def print_tsv(self, fields):
        dump_text = '\t'.join(fields) + "\n"
        self.put(self.frame_start, self.es_block, self.out_binary, [2, dump_text.encode()])

    def handle_checksum(self, value):
        self.putx([1, ['Checksum  (%02X)' % (value), 'Checksum', 'Chksum']])

        is_tester_request = self.source_id >= 0xF0 and self.source_id <= 0xFD

        valid_checksum = self.checksum_is_valid(self.frame, value)
        if not valid_checksum:
            self.putx([2, ['Checksum invalid', 'Chksum invalid', '!CHK']])

        else:
            # Broadcast this data
            self.put(self.frame_start, self.frame_end, self.out_python, {
                'dest': self.destination_id,
                'src': self.source_id,
                'data': self.frame[2:]
            })

        # Broadcast full frame to binary decoders
        self.put(self.frame_start, self.es_block, self.out_binary, 
                 [0 if is_tester_request else 1, bytes(self.frame)])
        
        hexdump = ''
        for b in self.frame[3:]:
            hexdump += '%02X ' % (b)
        
        if not self.tsv_header:
            self.tsv_header = True
            
            self.print_tsv([
                'Name',
                'Source device ID',
                'Destination device ID',
                'Frame data',
                'Checksum'
            ])

        self.print_tsv([
            'Tester' if is_tester_request else 'Device',
            '%02X' % self.source_id,
            '%02X' % self.destination_id,
            hexdump,
            'Valid' if valid_checksum else 'Invalid'
        ])
        
        self.frame.clear()
        self.fsm.transit(KlineFsm.State.Header)

    def handle_error(self, dummy):
        self.putx([3, ['Error', 'Err', 'E']])

    def checksum_is_valid(self, data, expected_checksum):
        checksum = 0
        for d in data:
            checksum += d

        return checksum % 0x100 == expected_checksum % 0x100

    def decode(self, ss, es, data):
        ptype, rxtx, pdata = data

        self.ss_block, self.es_block = ss, es

        if ptype != 'DATA':
            return

        # We're only interested in the byte value (not individual bits).
        pdata = pdata[0]

        handler = getattr(self, 'handle_%s' % self.fsm.state.lower())
        handler(pdata)
