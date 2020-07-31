#include "pch.h"
#include <iostream>
#include <algorithm>
#include <set>
#include <vector>

using namespace std;

struct Coord
{
    int x;
    int y;

    bool operator==(const Coord& c) {
        return x == c.x && y == c.y;
    }
    Coord() {
        x = 0;
        y = 0;
    }
    Coord(int x, int y) {
        this->x = x;
        this->y = y;
    }
};

struct Point
{
    int x;
    int y;
    Point* next;

    Point(int x, int y) {
        this->x = x;
        this->y = y;
        next = nullptr;
    }
};

struct Node
{
    Coord coord;
    Node* parent;
    int g;
    int h;
    int f;
    Node(Coord coord) {
        this->coord.x = coord.x;
        this->coord.y = coord.y;
        this->parent = 0;
        this->g = 0;
        this->h = 0;
        this->f = 0;
    }
    Node(Coord coord, Node* parent) {

        this->coord.x = coord.x;
        this->coord.y = coord.y;
        this->parent = parent;
        this->g = 0;
        this->h = 0;
        this->f = 0;
    }
    Node(Coord coord, Node* parent, int g, int h) {

        this->coord.x = coord.x;
        this->coord.y = coord.y;
        this->parent = parent;
        this->g = g;
        this->h = h;
        this->f = g + h;
    }
};

class HeapCompare_f
{
public:

    bool operator() (const Node* x, const Node* y) const
    {
        return x->f > y->f;
    }
};

class AStar
{
public:
    int* world_map;
    int width;
    int height;
    int bar_value;
    int direct_cost;
    int oblique_cost;
    int set_path_value;
    Node** neighbors;
    Node** map_cache;
    vector<Node*> open_ls;
    vector<Node* > close_ls;
    AStar(int* world_map, int width, int height, int bar_value, int dircect_cost, int oblique_cost, int set_path_value);
    ~AStar();
    Node* new_node(Node* parent, Coord end, int x, int y, int cost);
    void get_neighbors(Node* parent, Coord coord);
    Point* find_path(Coord start, Coord end);
    void free_ls();
};

AStar::AStar(int* world_map, int width, int height, int bar_value, int direct_cost, int oblique_cost, int set_path_value)
{
    this->world_map = world_map;
    this->width = width;
    this->height = height;
    this->bar_value = bar_value;
    this->direct_cost = direct_cost;
    this->oblique_cost = oblique_cost;
    this->map_cache = new Node * [(size_t)width * (size_t)height];
    this->neighbors = new Node * [8];
    this->set_path_value = set_path_value;
}


AStar::~AStar()
{
    delete[] map_cache;
    delete[] neighbors;
}

Node* AStar::new_node(Node* parent, Coord end, int x, int y, int cost)
{
    if (x < 0 || x >= width || y < 0 || y >= height)
    {
        return nullptr;
    }
    if (world_map[y * width + x] >= bar_value) {
        return nullptr;
    }
    int H = abs(end.x - x) * direct_cost + abs(end.y - y) * direct_cost;
    int G = parent->g + cost;
    Node* node = new Node(Coord(x, y), parent, G, H);
    return node;
}

void AStar::get_neighbors(Node* parent, Coord end)
{
    Coord cd = parent->coord;
    Node* up, * down, * left, * right, * left_up, * right_up, * left_down, * right_down;
    up = new_node(parent, end, cd.x, cd.y - 1, direct_cost);
    down = new_node(parent, end, cd.x, cd.y + 1, direct_cost);
    left = new_node(parent, end, cd.x - 1, cd.y, direct_cost);
    right = new_node(parent, end, cd.x + 1, cd.y, direct_cost);
    left_up = new_node(parent, end, cd.x - 1, cd.y - 1, oblique_cost);
    right_up = new_node(parent, end, cd.x + 1, cd.y - 1, oblique_cost);
    left_down = new_node(parent, end, cd.x - 1, cd.y + 1, oblique_cost);
    right_down = new_node(parent, end, cd.x + 1, cd.y + 1, oblique_cost);
    neighbors[0] = up;
    neighbors[1] = down;
    neighbors[2] = left;
    neighbors[3] = right;
    neighbors[4] = left_up;
    neighbors[5] = right_up;
    neighbors[6] = left_down;
    neighbors[7] = right_down;
}

void AStar::free_ls() {
    typename vector< Node* >::iterator iter_node;
    for (iter_node = open_ls.begin(); iter_node != open_ls.end(); iter_node++)
    {
        if (*iter_node != nullptr)
            delete (*iter_node);
    }
    for (iter_node = close_ls.begin(); iter_node != close_ls.end(); iter_node++)
    {
        if (*iter_node != nullptr)
            delete (*iter_node);
    }
    open_ls.clear();
    close_ls.clear();
}

Point* AStar::find_path(Coord start, Coord end)
{
    memset(map_cache, 0, (size_t)width * (size_t)height);
    Node* start_node = new Node(start);
    open_ls.push_back(start_node);
    make_heap(open_ls.begin(), open_ls.end(), HeapCompare_f());
    push_heap(open_ls.begin(), open_ls.end(), HeapCompare_f());
    map_cache[start.y * width + start.x] = start_node;
    Node* cur_node = nullptr;
    while (true)
    {
        if (open_ls.empty()) {
            printf("failed.\n");
            break;
        }
        cur_node = open_ls.front();
        pop_heap(open_ls.begin(), open_ls.end(), HeapCompare_f());
        open_ls.pop_back();
        if (cur_node->coord == end) {
            printf("success.\n");
            break;
        }
        get_neighbors(cur_node, end);
        for (size_t i = 0; i < 8; i++)
        {
            Node* node = neighbors[i];
            if (node == nullptr) continue;
            if (map_cache[node->coord.y * width + node->coord.x] != 0)
            {
                delete node;
                continue;
            }
            open_ls.push_back(node);
            push_heap(open_ls.begin(), open_ls.end(), HeapCompare_f());
            map_cache[node->coord.y * width + node->coord.x] = node;
        }
        close_ls.push_back(cur_node);
        map_cache[cur_node->coord.y * width + cur_node->coord.x] = cur_node;
    }
    Point* last = nullptr;
    if (cur_node == nullptr) goto end;
    last = new Point(cur_node->coord.x, cur_node->coord.y);
    printf("x:%d,y:%d\n", cur_node->coord.x, cur_node->coord.y);
    while (true)
    {
        if (set_path_value > 0)
            world_map[cur_node->coord.y * width + cur_node->coord.x] = set_path_value;
        cur_node = cur_node->parent;
        if (!cur_node) { break; }
        printf("x:%d,y:%d\n", cur_node->coord.x, cur_node->coord.y);
        Point* t = new Point(cur_node->coord.x, cur_node->coord.y);
        t->next = last;
        last = t;
    }
end:
    free_ls();
    return last;
}

extern "C" {
    __declspec(dllexport)void CreateAStar(int* world_map, int width, int height, int bar_value, int direct_cost, int oblique_cost, int set_path_value);
    __declspec(dllexport)Point* FindPath(int start_x, int start_y, int end_x, int end_y);
    __declspec(dllexport)void AStarFree();
}

AStar* finder=0;

void CreateAStar(int* world_map, int width, int height, int bar_value, int direct_cost, int oblique_cost, int set_path_value) {
    std::cout << "Hello CreateAStar!\n";
    if (finder != 0) delete finder;
    finder = new AStar(world_map, width, height, bar_value, direct_cost, oblique_cost, set_path_value);
}

Point* FindPath(int start_x,int start_y,int end_x,int end_y) {
    if (finder == 0) return 0;
    std::cout << "Hello FindPath!\n";
    return finder->find_path(Coord(start_x, start_y), Coord(end_x, end_y));
}

void AStarFree() {
    if (finder == 0) return;
    std::cout << "Hello AStarFree!\n";
    delete finder;
}

int main()
{

    int MAP_WIDTH = 20;
    int MAP_HEIGHT = 20;

    int BarValue = 9;
    int AreaValue = 1;


    int direct_cost = 10;
    int oblique_cost = 14;

    int is_direction = 0;

    int map[] = {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // 00
        1, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 1,   // 01
        1, 9, 9, 1, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,   // 02
        1, 9, 9, 1, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,   // 03
        1, 9, 1, 1, 1, 1, 9, 9, 1, 9, 1, 9, 1, 1, 1, 1, 9, 9, 1, 1,   // 04
        1, 9, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,   // 05
        1, 9, 9, 9, 9, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 1, 1, 1,   // 06
        1, 9, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1,   // 07
        1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,   // 08
        1, 9, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1,   // 09
        1, 9, 1, 1, 1, 1, 9, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // 10
        1, 9, 9, 9, 9, 9, 1, 9, 1, 9, 1, 9, 9, 9, 9, 9, 1, 1, 1, 1,   // 11
        1, 9, 1, 9, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,   // 12
        1, 9, 1, 9, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,   // 13
        1, 9, 1, 1, 1, 1, 9, 9, 1, 9, 1, 9, 1, 1, 1, 1, 9, 9, 1, 1,   // 14
        1, 9, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,   // 15
        1, 9, 9, 9, 9, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 1, 1, 1,   // 16
        1, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 9, 9, 1, 9, 9, 9, 9,   // 17
        1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,   // 18
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // 19
    };
    AStar a = AStar(map, MAP_WIDTH, MAP_HEIGHT, BarValue, direct_cost, oblique_cost, 7);
    a.find_path(Coord(0, 0), Coord(3, 3));
    for (int i = 0; i < MAP_HEIGHT; i++)
    {
        for (int j = 0; j < MAP_WIDTH; j++)
        {
            printf("%d  ", map[i * MAP_WIDTH + j]);
        }
        cout << endl;
    }
}





BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}

