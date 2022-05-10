# Mejoras de los Motores Fı́sicos: ODE y Bullet

Este proyecto contiene el código fuente desarrollado para el
Proyecto Fin de Máster titulado "Mejoras de los Motores físicos:
ODE y Bullet".

- Autor: Javier de la Puente Alonso.
- Tutor: Juan José Escribano Ródenas.
- Universidad Nacional de Educación a Distancia.
- Máster de Investigación en Ingeniería de Software y Sistemas Informáticos.
- Año 2020/2021.
- [Enlace a la Memoria.](http://www.issi.uned.es/Master_ISSI/WebMISSI/RepositorioTFM/2021/21F_Memoria_TFdM_ISW_TipoA_J_DeLaPuente_Alonso.pdf)


# Introducción

## Obugre

Obugre es un API común para los motores físicos ODE y Bullet, que además
permite la visualización de una escena en distintas instancias de motores
físicos. Aunque aún es un prototipo incompleto, permite realizar con facilidad
un análisis comparativo de los motores físicos sobre distintos escenarios. 

Es importante tener en cuenta que si el mismo motor físico se ejecuta
dos veces seguidas, los tiempos de rendimiento pueden no ser fiables
por el uso de las cachés.  Esto ocurre por ejemplo en 
performance_vclip_ode_bullet.cpp, donde se recomienda que para 
obtener los datos de rendimiento se ejecute cada escenario en cada motor
de forma separada.

    
## Algoritmo V-Clip y visualizador V-Clip. 

El algoritmo V-Clip (Mirtich, 1998) es un algoritmo que permite
encontrar las características más cercanas de dos poliedros convexos,
o en su caso, dar un testigo de la penetración entre los poliedros.

Además del algoritmo, en este proyecto se encuentra una herramienta 
gráfica, obvclip, que permite analizar la evolución del estado del
algoritmo. Un ejemplo de su uso se encuentra por ejemplo 
en obvclip/memoria/analisis1.cpp.

# Instalación


## Requisitos
El proyecto ha sido compilado y ejecutado con éxito en los sistemas ope-
rativos Ubuntu 20.04 LTS y Windows 10 con Visual Studio, ambos sobre
arquitectura x64.
Además de CMake, es necesario un compilador con soporte para C++11
y la posibilidad de compilar los proyectos QHull, Ogre, ODE y Bullet. Ya
que ODE solo está soportado en Linux, Windows y macOS, no es posible
utilizar este proyecto en otras plataformas.

## En Ubuntu 20.04 LTS
* Se instalarán las dependencias con el comando: “apt install cmake
  build-essential libxaw7-dev libxrandr-dev libsdl2-dev libzzip-dev”.
* Se deberá descomprimir el fichero que contiene el código fuente o 
  clonarse desde el repositorio de git correspondiente.
* Se deberá acceder al directorio principal del proyecto.
* En el caso de que el proyecto no se haya obtenido de git, será necesario
  ejecutar el comando “bash clone external.sh” para obtener las dependencias
  Eigen, QHull, ODE, Google Test, Bullet y Ogre. Algunas de estas dependencias
  han sido ligeramente modificadas para este proyecto, por lo que se descargan
  de la cuenta de GitHub del autor.
* Se creará el directorio de compilación. “mkdir build && cd build”
* Se invocará CMake “cmake ..”
* Opcionalmente se cambiarán las opciones de compilación con cmake o
  con una herramienta gráfica como “cmake-gui”.
* Con el comando “make” se compilará el código fuente.

Una vez compilado correctamente el proyecto, en el directorio “obvclip” se
encuentran los ejemplos de V-Clip sin utilizar motores fı́sicos. Como ejemplo,
desde el directorio “build” se puede ejecutar el comando “./obvclip/analisis1”
que permite iterar por los estados de V-Clip entre dos cuerpos.

En el directorio “obugre” se encuentran los ejecutables que utilizan la
aplicación Obugre para simular los motores fı́sicos. Como ejemplo, el coman-
do “./obugre/comparativa1” ejecutará la prueba 1 de este proyecto.

## En Windows 10

Instalar https://www.microsoft.com/en-us/download/details.aspx?id=6812

Además, para compilar en modo Release, es necesario
que Ogre utilice el fichero que está en extern/SDL2-2.0.10.tar.gz.
Se debe configurar en extern/ogre/CMake/Dependencies.cmake:186
para que lo baje de algún sitio web. Si no es modo Release,
el proyecto va muy lento en Windows.


# Guías de uso

## Herramienta visualización V-Clip. Directorio obvclip.

* Tecla ’1’. Muestra el wireframe.
* Tecla ’2’. Muestra cuerpos sólidos.
* Tecla ’3’. Muestra nombres de cuerpos, vértices y caras, los sistemas
  de coordenadas y las normales de las caras.
* Tecla ’4’. Muestra los planos de Voronoi “V-E” del estado actual.
* Tecla ’5’. Muestra los planos de Voronoi “E-F” del estado actual.
* Tecla ’6’. Muestra los contactos. No funciona si no es un estado final
  de V-Clip.
* Tecla ’7’. Muestra los contactos en modo simple o con algoritmo geométrico.
* Tecla ’p’. Imprime en pantalla la estructura de datos half-edge.
* Tecla ’c’. Avanza al siguiente estado de V-Clip.
* Tecla ’a’. Imprime en pantalla el estado actual de V-Clip.
* Tecla ’n’. Si hay más de dos cuerpos en la escena, avanza al siguiente
  par de cuerpos.
* Tecla ’d’. Guarda un fichero png con la captura de pantalla.

Para algunos de estos comandos será necesario estar en un estado de
V-Clip válido, para lo que es necesario haber invocado al comando ’c’.

## Obugre. Directorio obugre.

* Tecla Escape. Sale de la animación.
* Tecla Espacio. Para/Arranca la animación.
* Tecla ’c’. Para la animación en caso de colisión.
* Tecla ’1’. Muestra los sistemas de coordenadas de los cuerpos.
* Tecla ’2’. Muestra información textual de los cuerpos, nombre del cuerpo, 
  energı́a cinética, norma del momento lineal y norma del momento
  angular.
* Tecla ’3’. Muestra los vectores de momento angular y momento linear
  de los cuerpos.
* Tecla ’4’. Muestra los sistemas de coordenadas de los motores.
* Tecla ’5’. Muestra información textual de los motores, nombre del motor, 
  energı́a cinética, norma del momento lineal y norma del momento
  angular.
* Tecla ’6’. Muestra los vectores de momento angular y momento linear
  de los motores, con respecto al origen.
* Tecla ’7’. Muestra los contactos.
* Tecla ’8’. Cambia entre proyección en perspectiva y proyección 
  ortográfica.
* Tecla ’9’. En proyección ortográfica, amplia el frustrum de la cámara.
* Tecla ’0’. En proyección ortográfica, disminuye el frustrum de la cámara.
* Tecla ’s’. Guarda un fichero png con la captura de pantalla.

## Uso algoritmo V-Clip

El código que implementa el algoritmo V-Clip está en los directorios
obvclip/include y obvclip/src.
La clase ConvexPolytope representa la geometrı́a de un poliedro convexo.
La clase WorldConvexPolytope representa un ConvexPolytope con una pose.
En el fichero de cabecera polytope_examples.hpp se encuentran funciones
de ayuda para crear algunos tipos de poliedros. 

Por ejemplo, y de forma
extensa, para crear una pirámide con base pentagonal de altura 1 y radio de
la base 2 a partir de puntos, y darle una pose:

    std::vector<OB::Point> puntos = get_pyramid_points(5,1,2);
    std::shared_ptr<OB::ConvexPolytope> piramide = 
        std::make_shared<OB::ConvexPolytope>();
    piramide->generate_from_vertices(puntos);
    OB::WorldConvexPolytope wcp{"piramide", piramide};
    OB::Transform t = OB::Translation(0, -3 ,0);
    t *= OB::AngleAxis(OB::PI * 0.2, OB::Vector::UnitX());
    wcp.set_pose(t);

Una vez creados dos WorldConvexPolytope , se puede invocar sobre ellos el
algoritmo V-Clip. Para ello será necesario o invocar a la función VClip::solve
con un testigo al azar o al operador VClip::operator con una instancia de
caché VClipCache:

    OB::WorldConvexPolytope wcp0 = ...
    OB::WorldConvexPolytope wcp1 = ...
    OB::VClip vclip{wcp0, wcp1};
    VClipWitness witness{VClipWitness::FirstVertices()};
    VClipResult result = vclip.solve(witness);

La estructura VClipResult contiene el resultado del algoritmo V-Clip. En-
tre otros campos son los testigos de menor distancia o de penetración, si
hay o no penetración y la distancia entre los testigos. Con la salida de V-
Clip se pueden invocar las funciones del fichero de cabecera contacts.hpp que
calculan puntos de contactos, normales y distancias entre los poliedros.

Además, en el fichero polytope.hpp se encuentran multitud de funcio-
nes que pueden resultar útiles para el cálculo de distancias entre carac-
terı́sticas. Algunas de estas son absdist_edge_face, absdist_between_features,
dist_edge_edge , closest_point_to_line ...

## Uso Herramienta visualización V-Clip

El código fuente se encuentra en obvclip/tools y ejemplos de su uso en
obvclip/apps y obvclip/memoria . Para ejecutar la herramienta, una vez creados
los objectos WorldConvexPolytope , se puede utilizar como referencia el código
siguiente:

    OB::OgreVClipApp app;
    app.initApp();
    app.addWorldConvexPolytope(...);
    ...
    app.addWorldConvexPolytope(...);
    app.getRoot()->startRendering();
    app.closeApp();

    
## Uso Obugre

Obugre está planteado como un API común a los motores fı́sicos al estilo
PAL, junto con la posibilidad de visualización simultánea de varios motores.

Obugre no es un proyecto finalizado, al haberse implementado solo las
caracterı́sticas necesarias para la realización de este proyecto. Igualmente,
aspectos como rendimiento no han sido considerados en su diseño ya que
es una herramienta para comparar motores y no para su uso en productos
finales. Se describe en este apartado los principios de su funcionamiento.

La clase principal de Obugre es la clase System . Una vez creada una ins-
tancia, será necesario añadir motores fı́sicos, cuerpos con su geometrı́a y
restricciones. Se podrá optar qué cuerpos y que restricciones ser desean uti-
lizar en qué motores. Una vez configurado el escenario y los motores fı́sicos,
la clase Looper ejecutará la simulación con los motores deseados.

Para poder interactuar con los motores fı́sicos durante la simulación es
posible utilizar los callbacks definidos en EngineListener , que permiten a la
aplicación obtener información o modificar aspectos del escenario. Un ejemplo
de este uso es la clase EngineLogger, utilizada para obtener información de
tiempos de ejecución o de propiedades fı́sicas de los motores.

Algunas de las funciones más útiles a la hora de utilizar Obugre son las
siguientes:
* Engine::add_listener. Añade un listener al motor deseado.
* Engine::set_use_vclip. Utiliza el algoritmo V-Clip para colisiones entre
  poliedros convexos.
* Looper::set_simulation_speed. Cambia la velocidad de la visualización 
  de la simulación.
* Looper::set_pause_on_collisions. La simulación se detiene cuando en
  algún motor hay una colisión.

Para ejemplos completos de uso se pueden consultar los ficheros en los
directorios obugrep/apps y obugre/memoria . Un ejemplo simple de caso de uso:


    System system{};
    system.set_default_gravity(Vector(0.0, -9.8, 0.0));

    system.create_engine("ode", EngineType::Ode);
    system.create_engine("bullet", EngineType::Bullet);

    Plane ground_plane{Vector(0, 1, 0).normalized(), 0};
    Body& ground =  system.create_body("ground");
    ground.get_initial_state().set_position(Vector(0,0,0));
    ground.set_static(true);

    Body& box = system.create_body("box");
    BoxShape& box_shape = box1.add_box_shape(Vector(3,4,5));
    box.get_initial_state().set_position(Vector(-10,0,0));
    box.get_initial_state().set_angular_velocity(Vector(0, -10, 0));


    Body& sphere = system.create_body("sphere");
    SphereShape& sphere_shape = sphere.add_sphere_shape(5);
    sphere.get_initial_state().set_position(Vector(-10,0,0));
    sphere.get_initial_state().set_angular_velocity(Vector(0, -10, 0));

    HingeConstraint& constraint =
        system.create_hinge_constraint("constraint", "box", "sphere",
                                       Vector(0, 1, 0), Point(2, 0, 0));

    system.insert_all_bodies_in_all_engines();
    system.insert_all_constraints_in_all_engines();

    Looper looper{system};
    looper.add_engine("ode", Vector{-40, 0, 0});
    looper.add_engine("bullet", Vector{-15, 0, 0});
    looper.loop();

