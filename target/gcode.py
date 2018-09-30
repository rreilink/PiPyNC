

class GcodeRunner:
    axes = 'XYZEABC'
    steps_per_mm = [400 * 32 / 40, 400 * 32/40, 400*32/2, 100,100,100,100]

    def __init__(self):
        self.source = iter([])
        self.reset()
        
    def reset(self):
        self.state = dict(relative = False, position = [0]*7, position_steps = [0] * 7, feed = None)

    def setsource(self, source):
        self.source = source
    
    
    def move(self, target, feed):
        import app
        while True:
            try:
                app.queuemove((target[0], target[1]), 100)
            except RuntimeError:
                pass
            else:
                break
    
    def runner(self):
        state = self.state
        
        while True:
            line = next(self.source)
            line = line.partition(';')[0]
            if line.startswith('M117'):
                print(line[5:])
                continue
            
            words = {}
            for word in line.split():
                words[word[0]] = float(word[1:] or '0')
            
            M = words.get('M')
            G = words.get('G')
            F = words.get('F')
            if F is not None:
                state['feed'] = F
            
            if M is not None and G is not None:
                print('Error: both M and G word in line {line!r}')
            else:
            
                if G == 0 or G == 1:
                    if state['relative']:
                        base = state['position']
                    else:
                        base = [0] * len(self.axes)
                    
                    target = state['position'].copy()
                    for i, axis in enumerate(self.axes):
                        value = words.get(axis)
                        if value is not None:
                            target[i] = base[i] + value
                    
                    if target != state['position']:
                        self.move(target, self.state['feed'] / 60) # convert mm/min to mm/s
                        
                    state['position'] = target 
                    
                elif G==90:
                    state['relative'] = False
                elif G==91:
                    state['relative'] = True
                elif G==92:
                    for axis, i in enumerate(self.axes):
                        value = words.get(axis)
                        if value is not None:
                            state[position][i] = value
                elif G==4:
                    print('Wait')
                elif G==28:
                    print('Home')
                elif G!=None:
                    print(f'Unknown G-Code {G} in line {line!r}')
                
                if M==84:
                    print('Stop idle hold')
                elif M==80:
                    print('Power on')
                elif M==82:
                    print('Extruder absolute')
                    
                elif M==104:
                    print('Set extruder temperature')
                elif M==109:
                    print('Set extruder and wait')
                elif M==140:
                    print('Set bed temperature')
                elif M==106:
                    print('Fan on')
                elif M==107:
                    print('Fan off')
                elif M==190:
                    print('Wait bed temp')    
                elif M!=None:
                    print(f'Unknown M-Code {M} in line {line!r}')

        yield


def runfile(filename):
    g = GcodeRunner()
    with open(filename) as file:
        g.setsource(file)
        r = g.runner()
        for i in r:
            pass
    
if __name__ == '__main__':
    runfile('/boot/app/test.gcode')
            
    