//
// Created by salabeta on 24/11/20.
//

#ifndef GOTOXY_GRID_H
#define GOTOXY_GRID_H

#include <QGraphicsItem>

template<typename HMIN, HMIN hmin, typename WIDTH, WIDTH width, typename TILE, TILE tile>
class Grid
{
    int w=width;
    std::vector<std::tuple<int, int>> listaVecinos{ {-1,-1}, {0,-1}, {1,-1}, {-1,0}, {1,0}, {-1,1}, {0,1}, {-1,1} };
    public:
    int tam = 0;
        Grid()
        {
            array.resize((int)(width/tile));
            for (auto &row : array)
                row.resize((int)(width/tile));
            int k=0;
            for (int i = hmin; i < width/2; i += tile, k++)
            {
                int l=0;
                for (int j = hmin; j < width/2; j += tile, l++)
                {                               
                    array[k][l] = Value{false, nullptr,nullptr, i, j,k,l,-1};
                }
            }
            tam = (int)(width/tile);
        };

        struct Value
        {
            bool occupied = false;
            QGraphicsRectItem *paint_cell = nullptr;
            QGraphicsTextItem *text_cell = nullptr;
            int cx, cy;
            int k,l;
            int dist; //dist vecinos
        };
public:
        std::vector<std::vector<Value>> array;

        // void create_graphic_items(QGraphicsScene &scene)
        // {
        //     for (auto &row : array)
        //         for (auto &elem : row)
        //         {
        //             elem.paint_cell = scene.addRect(-tile / 2, -tile / 2, tile, tile, QPen(QColor("White")),
        //                                             QBrush(QColor("White")));
        //             elem.paint_cell->setPos(elem.cx, elem.cy);
        //         }
        // }
        void create_graphic_items(QGraphicsScene &scene)
        {
            scene.clear();
            auto fondo = QColor("LightGreen"); fondo.setAlpha(40);
            QFont font("Bavaria");
            font.setPointSize(40);
            font.setWeight(QFont::TypeWriter);
            for (int i = 0; i< tam; i++){
                for (int j = 0; j< tam; j++)
                {
                    array[i][j].paint_cell = scene.addRect(-tile / 2, -tile / 2, tile, tile, QPen(QColor("DarkGreen")), QBrush(fondo));
                    array[i][j].paint_cell->setPos(array[i][j].cx, array[i][j].cy);

                    array[i][j].text_cell = scene.addText(QString::number(array[i][j].dist), font);
                    array[i][j].text_cell->setPos(array[i][j].cx-tile/2, array[i][j].cy-tile/2);
                    // Get the current transform
                    QTransform transform(array[i][j].text_cell->transform());
                    qreal m11 = transform.m11();    // Horizontal scaling
                    qreal m12 = transform.m12();    // Vertical shearing
                    qreal m13 = transform.m13();    // Horizontal Projection
                    qreal m21 = transform.m21();    // Horizontal shearing
                    qreal m22 = transform.m22();    // vertical scaling
                    qreal m23 = transform.m23();    // Vertical Projection
                    qreal m31 = transform.m31();    // Horizontal Position (DX)
                    qreal m32 = transform.m32();    // Vertical Position (DY)
                    qreal m33 = transform.m33();    // Addtional Projection Factor
                    // Vertical flip
                    m22 = -m22;
                    // Write back to the matrix
                    transform.setMatrix(m11, m12, m13, m21, m22, m23, m31, m32, m33);
                    // Set the items transformation
                    array[i][j].text_cell->setTransform(transform);
                }
            }
        }

        //void update(){
        //    for (int i = 0; i< tam; i++){
        //        for (int j = 0; j< tam; j++){
        //            array[i][j].text_cell->setPlainText(QString::number(array[i][j].dist));
        //        }
        //    }
       // }
    
    /*
 * Inicializamos el array a false, osea, no ocupades.
 */

public:
    int get_width(){
        return w;
    }
    int transformarX(int x)
    {
      int i = 0;
      i = (int)((x/tile) + (this->tam/2)); 
      return i;
    }

    int transformarZ(int z)
    {
        int j = 0;
        j = (int)((z/tile) + (this->tam/2));
        return j;
    }
    /**
     * modificamos en funcion de v la coordenada x,z
     * @param x
     * @param z
     * @param v
     */
    void set_Value(int x, int z, bool v)
    {
       //this->array[x][z] = v;
       //auto [i, j] = transformar(x,z);
       //array[i][j].occupied = v;
       //if(v)
       //     array[i][j].paint_cell->setColor(QColor());
       int i,j = 0;
       i = transformarX(x);
       j = transformarZ(z);
       
       if(i>=0 && j>=0 && i<tam && j<tam){
            array[i][j].occupied = v;
            if(v)
            {
                array[i][j].paint_cell->setBrush(QColor("Red"));
            }
       }
       

    }
    /**
     * devolvemos el valor de la coordenada x,z
     * @param x
     * @param z
     * @return
     */
    bool get_value(int x, int z)
    {
        //auto [i, j] = transformar(x,z);
        //return  this->array[x][z];
        int i,j = 0;
        i = transformarX(x);
        j = transformarZ(z);
        if(i>=0 && j>=0 && i<tam && j<tam){
            return array[i][j].occupied;
        }
        return {};
    }
    int get_distance(int x, int z){
        int i,j = 0;
        i = transformarX(x);
        j = transformarZ(z);
        return array[i][j].dist;
    }
    void set_distance(int x, int z,int dist){
        int i,j = 0;
        i = transformarX(x);
        j = transformarZ(z);
        if(i>=0 && j>=0 && i<tam && j<tam){
            array[i][j].dist=dist;
        }
    }

    void pintarBordes()
    {
        for(int i = -width/2;i < width/2;i++)
        {
            set_Value(i,-width/2,true);
            set_Value(i,width/2-1,true);
            set_Value(-width/2,i,true);
            set_Value(width/2-1,i,true);
        }
    }
    void reset_cell_distances(){
        for (int i = 0; i< tam; i++){
            for (int j = 0; j< tam; j++){
                array[i][j].dist = -1;
            }
        }
    }
    bool get_occupied(int k, int l){
        return array[k][l].occupied;
    }
    int get_dist(int k, int l){
        return array[k][l].dist;
    }
    void set_dist(int k, int l,int dist){
        array[k][l].dist=dist;
    }
    Value get_cell(int k, int l){
        return array[k][l];
    }
    Value get_cell2(int k, int l){
        int i,j = 0;
        i = transformarX(k);
        j = transformarZ(l);
        if(i>=0 && j>=0 && i<tam && j<tam){
            return array[i][j];
        }
        return {};
    }
    void updateText(int k, int l)
    {
        qDebug() << "antes";
        array[k][l].text_cell->setPlainText("0");
        qDebug() << "después";

    }
};


#endif //GOTOXY_GRID_H
