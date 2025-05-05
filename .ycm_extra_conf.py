def Settings( **kwargs ):
  return {
    'flags': ['-x', 'c++', '-Wall', '-pedantic',
    '-std=c++23',
    '-I/usr/include',
    '-Ithirdparty/glm',
    ],
  }
