#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <random>
#include <unordered_set>
#include <chrono>
#include <thread>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace sf;


const int GRID_SIZE = 30;
const int CELL_SIZE = 25;
const int WINDOW_SIZE = GRID_SIZE * CELL_SIZE;    
const int STEP_DELAY_MS = 20;                     
const int PRESET_MAP_DIM = 10;                 


const int MAP[PRESET_MAP_DIM][PRESET_MAP_DIM] = {
    { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, //мря карта для 2 варианта, единички это стены
    { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 },
    { 1, 0, 1, 0, 0, 0, 1, 0, 0, 1 },
    { 1, 0, 1, 0, 0, 0, 0, 0, 0, 0 },
    { 1, 1, 0, 1, 0, 0, 0, 0, 0, 0 },
    { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0 },
    { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0 },
};


enum CellType {
    EMPTY,     
    OBSTACLE,  
    START,     
    END,       
    PATH,      
    VISITED,   
    FRONTIER   
};


struct Cell {
    int x, y;
    CellType type;
    double f, g, h;            
    Cell* parent;               
    vector<Cell*> sosedi; 

    Cell(int x, int y) : x(x), y(y), type(EMPTY), f(0), g(0), h(0), parent(nullptr) {}

   
    bool operator==(const Cell& other) const {
        return x == other.x && y == other.y;
    }

  
    void get_soseds(vector<vector<Cell>>& grid) {
        sosedi.clear();
        int dir_x[] = {0, 0, -1, 1};
        int dir_y[] = {-1, 1, 0, 0};
        for (int i = 0; i < 4; i++) {
            int new_x = x + dir_x[i];
            int new_y = y + dir_y[i];
            if (isValid(new_x, new_y) && grid[new_x][new_y].type != OBSTACLE) {
                sosedi.push_back(&grid[new_x][new_y]);
            }
        }
    }

   
    static bool isValid(int x, int y) {
        return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
    }
};


double heuristic(const Cell& a, const Cell& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

//Создаём пустую сетку GRID_SIZE x GRID_SIZE
vector<vector<Cell>> create_grid() {
    vector<vector<Cell>> grid;
    for (int i = 0; i < GRID_SIZE; i++) {
        vector<Cell> row;
        for (int j = 0; j < GRID_SIZE; j++) {
            row.push_back(Cell(i, j));
        }
        grid.push_back(row);
    }
    return grid;
}

//Загружаем мою карту 10x10 и ставим старт/финиш
void load_map(vector<vector<Cell>>& grid, Cell*& start_cell, Cell*& goal_cell) {
    //сначала всё очищаем
    for (int i = 0; i < GRID_SIZE; i++)
        for (int j = 0; j < GRID_SIZE; j++) {
            grid[i][j].type = EMPTY;
            grid[i][j].f = grid[i][j].g = grid[i][j].h = 0;
            grid[i][j].parent = nullptr;
        }

    //расставляем стены по шаблону
    for (int row = 0; row < PRESET_MAP_DIM; row++)
        for (int col = 0; col < PRESET_MAP_DIM; col++)
            if (MAP[row][col] == 1)
                grid[col][row].type = OBSTACLE;

    //фиксированные старт и финиш
    start_cell = &grid[0][1];
    goal_cell = &grid[9][9];
    start_cell->type = START;
    goal_cell->type = END;
}

//Отрисовка всей сетки в окне
void draw(RenderWindow& app_window, vector<vector<Cell>>& grid) {
    for (int col = 0; col < GRID_SIZE; ++col) {
        for (int row = 0; row < GRID_SIZE; ++row) {
            RectangleShape tile(Vector2f(CELL_SIZE - 1, CELL_SIZE - 1));
            tile.setPosition(col * CELL_SIZE, row * CELL_SIZE);

            //красим в зависимости от типа клетки
            switch (grid[col][row].type) {
            case EMPTY:    tile.setFillColor(Color::White); break;
            case OBSTACLE: tile.setFillColor(Color::Black); break;
            case START:    tile.setFillColor(Color(255, 165, 0)); break; //оранжевый
            case END:      tile.setFillColor(Color(64, 224, 208)); break; //бирюзовый
            case PATH:     tile.setFillColor(Color(128, 0, 128)); break; //фиолетовый
            case VISITED:  tile.setFillColor(Color::Red); break;
            case FRONTIER: tile.setFillColor(Color::Green); break;
            }

            app_window.draw(tile);
        }
    }
}

//Нужно для очереди с приоритетом: так сказал гугл
struct Comp {
    bool operator()(const Cell* a, const Cell* b) {
        return a->f > b->f;
    }
};

//Восстанавливаем путь от текущей клетки назад к старту по parent
vector<Cell*> build_path(Cell* current_cell) {
    vector<Cell*> path;
    while (current_cell != nullptr) {
        path.push_back(current_cell);
        current_cell = current_cell->parent;
    }
    reverse(path.begin(), path.end()); //переворачиваем, чтобы было от старта к фишишу
    return path;
}

//Сам алгоритм а* с анимацией шагов
vector<Cell*> a_star_algorithm(RenderWindow& app_window, vector<vector<Cell>>& grid,
                        Cell* start_cell, Cell* goal_cell) {

    //открытый список и множества для быстрой проверки принадлежности
    priority_queue<Cell*, vector<Cell*>, CellCostComparator> queue;
    unordered_set<Cell*> set;   //чтобы знать, есть ли клетка в открытом списке
    unordered_set<Cell*> set2;   //закрытый список

    //начальные оценки для старта
    start_cell->g = 0;
    start_cell->h = heuristic(*start_cell, *goal_cell);
    start_cell->f = start_cell->h;      //f = g + h = h

    queue.push(start_cell);
    set.insert(start_cell);

    int iters = 0;

    while (!queue.empty()) {
        //обрабатываем окошко (закрытие, Esc для прерывания)
        Event ev;
        while (app_window.pollEvent(ev)) {
            if (ev.type == Event::Closed) {
                app_window.close();
                return vector<Cell*>();
            }
            if (ev.type == Event::KeyPressed) {
                if (ev.key.code == Keyboard::Escape) {
                    return vector<Cell*>();   //пользователь прервал поиск
                }
            }
        }

        //берём клетку с наименьшим f из открытого списка
        Cell* current_cell = queue.top();
        queue.pop();
        set.erase(current_cell);

        //если это цель, путь найден
        if (current_cell == goal_cell) {
            cout << "Путь найден за " << iters << " шагов!" << endl;
            return build_path(current_cell);
        }

        //помечаем как посещённую (если не старт)
        if (current_cell->type != START) {
            current_cell->type = VISITED;
        }
        set2.insert(current_cell);

        //обновляем список соседей для текущей клетки
        current_cell->get_soseds(grid);

        //перебираем всех соседей
        for (Cell* sosed : current_cell->sosedi) {
            //если уже в закрытом списке, пропускаем
            if (set2.find(sosed) != set2.end()) {
                continue;
            }

            //новая стоимость пути от старта до соседа
            double new_cost = current_cell->g + 1.0; //шаг стоит 1

            //если соседа ещё нет в открытом списке, добавляем
            if (set.find(sosed) == set.end()) {
                sosed->parent = current_cell;
                sosed->g = new_cost;
                sosed->h = heuristic(*sosed, *goal_cell);
                sosed->f = sosed->g + sosed->h;
                if (sosed->type != END) {
                    sosed->type = FRONTIER;
                }
                queue.push(sosed);
                set.insert(sosed);
            }
            //если уже есть, но новый путь дешевле, обновляем оценки
            else if (new_cost < sosed->g) {
                sosed->parent = current_cell;
                sosed->g = new_cost;
                sosed->f = sosed->g + sosed->h;
                //приоритет в куче сам не обновится, но это упрощённый вариант,
                //для небольших полей работает нормально
            }
        }

        iters++;

        //перерисовываем поле и ждём, чтобы видеть процесс
        app_window.clear();
        draw(app_window, grid);
        app_window.display();
        this_thread::sleep_for(chrono::milliseconds(STEP_DELAY_MS));
    }

    //если открытый список опустел, а цель не нашли, пути нет
    return vector<Cell*>();
}

//Заполняем поле случайными препятствиями с заданной плотностью (по умолчанию 25%)
void obstacles(vector<vector<Cell>>& grid, float obstacle_density = 0.25f) {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<float> dis(0.0f, 1.0f);
    for (int i = 0; i < GRID_SIZE; i++)
        for (int j = 0; j < GRID_SIZE; j++)
            if (grid[i][j].type == EMPTY && dis(gen) < obstacle_density)
                grid[i][j].type = OBSTACLE;
}

int main() {
    //создаём окно
    RenderWindow app_window(VideoMode(WINDOW_SIZE, WINDOW_SIZE), "A* Algorithm Visualization");
    app_window.setFramerateLimit(60);

    vector<vector<Cell>> grid = create_grid();

    //старт, левый верхний угол, фмнищ, правый нижний
    Cell* start_cell = &grid[0][0];
    Cell* goal_cell = &grid[GRID_SIZE - 1][GRID_SIZE - 1];
    start_cell->type = START;
    goal_cell->type = END;

    //накидываем случайных стен
    obstacles(grid, 0.25f);

    bool path_computed = false;  //был ли уже найден путь
    bool use_preset_map = false; //используем ли карту с картинки
    vector<Cell*> final_path;

    cout << "=== Алгоритм а* ===" << endl;
    cout << "Управление:" << endl;
    cout << "SPACE - запустить алгоритм A*" << endl;
    cout << "R - сгенерировать новое поле" << endl;
    cout << "M - загрузить карту из изображения (10x10)" << endl;
    cout << "ESC - прервать выполнение алгоритма" << endl;

    while (app_window.isOpen()) {
        Event ev;
        while (app_window.pollEvent(ev)) {
            if (ev.type == Event::Closed) {
                app_window.close();
            }

            if (ev.type == Event::KeyPressed) {
                //пробел, запуск A*
                if (ev.key.code == Keyboard::Space && !path_computed) {
                    cout << "Запуск алгоритма A*";
                    if (use_preset_map) cout << " [карта из изображения]";
                    cout << "..." << endl;

                    //очищаем все временные пометки с прошлого запуска
                    for (int i = 0; i < GRID_SIZE; i++) {
                        for (int j = 0; j < GRID_SIZE; j++) {
                            if (grid[i][j].type == VISITED ||
                                grid[i][j].type == FRONTIER ||
                                grid[i][j].type == PATH) {
                                grid[i][j].type = EMPTY;
                            }
                            grid[i][j].f = 0;
                            grid[i][j].g = 0;
                            grid[i][j].h = 0;
                            grid[i][j].parent = nullptr;
                        }
                    }

                    //если включена карта из задания, заново расставляем стены
                    if (use_preset_map) {
                        for (int row = 0; row < PRESET_MAP_DIM; row++)
                            for (int col = 0; col < PRESET_MAP_DIM; col++)
                                if (MAP[row][col] == 1)
                                    grid[col][row].type = OBSTACLE;
                        start_cell->type = START;
                        goal_cell->type = END;
                    }

                    //сам поиск
                    final_path = run_astar(app_window, grid, start_cell, goal_cell);
                    path_computed = true;

                    if (!final_path.empty()) {
                        //раскрашиваем найденный путь
                        for (Cell* cell : final_path) {
                            if (cell->type != START && cell->type != END) {
                                cell->type = PATH;
                            }
                        }
                        cout << "Путь найден! Длина: " << final_path.size() << " ячеек" << endl;
                    } else {
                        cout << "Путь не найден!" << endl;
                    }
                }

                //клавиша R, сбросить на случайное поле
                if (ev.key.code == Keyboard::R) {
                    cout << "Генерация нового поля..." << endl;
                    use_preset_map = false;

                    grid = create_grid();
                    start_cell = &grid[0][0];
                    goal_cell = &grid[GRID_SIZE - 1][GRID_SIZE - 1];
                    start_cell->type = START;
                    goal_cell->type = END;

                    obstacles(grid, 0.25f);

                    path_computed = false;
                    final_path.clear();
                    cout << "Новое поле создано." << endl;
                }

                //клавиша M, загрузить мою карту
                if (ev.key.code == Keyboard::M) {
                    cout << "Загрузка карты из изображения (10x10)..." << endl;
                    use_preset_map = true;

                    grid = create_grid();
                    load_map(grid, start_cell, goal_cell);

                    path_computed = false;
                    final_path.clear();
                    cout << "Карта загружена. Старт: (0,1), Конец: (9,9)" << endl;
                    cout << "Нажмите SPACE для запуска A* по этой карте." << endl;
                }
            }
        }

        //рисуем все
        app_window.clear();
        draw(app_window, grid);
        app_window.display();
    }

    return 0;
}
