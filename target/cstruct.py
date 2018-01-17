cdef = """
typedef struct {
    int naxis;
    uint32_t step_mask[MAX_AXIS];
    uint32_t dir_mask[MAX_AXIS];
} stepper_config_t;
"""
import struct
import array

class CStruct:
    '''
    Python representation of a C data structure
    
    This could also be done using ctypes.Structure, but that is not (yet)
    available in PyPiOS
    
    definition is a string like "name:type,name2:type2"
    data is a bytearray that contains the data
    '''
    def __init__(self, definition, data):
        format = ''
        def_dict = {} # dictionary of attribute name -> (size, offset, type)
        for item in definition.split(','):
            name, sep, type = item.rpartition(':')
            
            # format = total format upto now, to calculate offset, taking into
            # account alignment          
            format += type
            
            size = struct.calcsize(type)
            offset = struct.calcsize(format) - size
            def_dict[name] = offset, size, type
            

        if struct.calcsize(format) != len(data):
            print(format, struct.calcsize(format), len(data))
            assert False
        
        self.__dict__['_CStruct__def'] = def_dict
        self.__dict__['_CStruct__data'] = data
        
    def __getattr__(self, attr):
        try:
            offset, size, type = self.__def[attr]
        except KeyError:
            raise AttributeError
        
        if type[0].isdigit():
            # For array-access, return a memoryview, such that the returned 
            # object can be modified, and changes will be effected in the
            # data
            value = memoryview(self.__data)[offset:offset+size].cast(type[1:])
        if not type[0].isdigit():
            value, = struct.unpack(type, self.__data[offset:offset+size])
        return value
        
    def __setattr__(self, attr, value):
        try:
            offset, size, type = self.__def[attr]
        except KeyError:
            raise AttributeError
        
        if not type[0].isdigit():
            value = (value,)
        
        self.__data[offset:offset+size] = struct.pack(type, *value)
        
    def __dir__(self):
        return self.__def.keys()
    
    def __repr__(self):
        def convert(s): # Convert memoryview to list for repr()
            return list(s) if isinstance(s, memoryview) else s
            
        return '\n'.join('%s:%s' % (n, convert(getattr(self, n))) for n in self.__def)
       
    
    
        
config_defi = 'naxis:I,step_mask:8I,dir_mask:8I,steps_per_mm:8f,machine_steps_per_mm:f,max_acceleration:f'
config_data = bytearray(108)
s = CStruct(config_defi, config_data)


