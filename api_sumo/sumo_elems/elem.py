"""
Basic object type to represent a sumo object in python.
一个sumo对象的基类
该基类实例在初始化时，确定名称、id，初始化一个属性列表；
后续，基类实例的任何属性的读写均只在属性列表中；不在列表中的属性无法新建。
"""

class Elem(object):
    def __init__(self,name,_id,d={},**args):
        super(Elem,self).__setattr__('name',name)
        super(Elem,self).__setattr__('id',_id)
        super(Elem,self).__setattr__('attr',d)
        
    def __set_attr__(self,k,v):
        '''Extra method to add extra attributes to an object'''
        super(Elem,self).__setattr__(k,v)
    
    def __setitem__(self,key,value):
        '''To set items with brackets []'''
        self.__setattr__(key,value)
        
    def __getitem__(self,key):
        '''To get items with brackets []'''
        return self.__getattr__(key)
            
    def __setattr__(self,key,value):
        '''To set items with point p.e.  node.x = 0 to set attribute x of node object to 0'''
        if key not in self.attr:
            raise AttributeError(\
            'Set attribute error in {} {}'.format(type(self).__name__,self.id))
        self.attr[key] = value
    
    def __getattr__(self,key):
        '''To get items with point'''
        try:
            return self.attr[key]
        except KeyError:
            raise AttributeError(\
            'Get attribute error in {} {}'.format(type(self).__name__,self.id))
    
    def __str__(self):
        '''Return a string defining an object'''
        return 'Object {} with id: {}'.format(type(self).__name__,self.id)
    
    def __repr__(self):
        '''Return a string in xml/sumo format defining this object'''
        strlist = ['\t<{0}'.format(self.name)]
        strlist.append('id="{}"'.format(self.id))
        for key in self.attr:
            if self.attr[key] != None:
                strlist.append('{}="{}"'.format(key,self.attr[key]))
        strlist.append('/>\n')
        return ' '.join(strlist)