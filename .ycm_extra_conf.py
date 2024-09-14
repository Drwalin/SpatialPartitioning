def Settings( **kwargs ):
  return {
    'flags': ['-x', 'c++', '-Wall', '-pedantic',
    '-std=c++20',
    '-I/usr/include',
    '-Ithirdparty/glm'
    ],
  }
