// Eurobot 2026 – Visual branching demo (SFML, 8x6 grid)
// Start at Nest Y, robot as yellow star, arrows = possible plans.
// Build (MSYS2 MinGW64):
//   C:\msys64\mingw64\bin\g++.exe -std=c++17 -O2 -g ai_decision_demo.cpp -o ai_decision_demo.exe ^
//     -I"C:\msys64\mingw64\include" -L"C:\msys64\mingw64\lib" ^
//     -lsfml-graphics -lsfml-window -lsfml-system -lopengl32 -lgdi32 -lwinmm -lws2_32
// Put table_bis.png + DejaVuSans.ttf next to the EXE.

#include <SFML/Graphics.hpp>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <filesystem>
#include <algorithm>
#include <cstdio>
using namespace std;
#include <queue>     // for std::priority_queue
#include <tuple>     // if you use std::tie somewhere


static const int COLS=8, ROWS=6, MAX_CARRY=3;
struct P{int x=0,y=0; bool operator==(const P&o)const{return x==o.x&&y==o.y;}};
static const map<string,P> PT={{"nest_y",{0,0}},{"nest_b",{7,0}},{"collection",{0,3}},{"pantry",{7,3}},{"thermo",{4,5}}};

static sf::Vector2f px(int cx,int cy){ return {cx*100.f+5.f,(float)((ROWS-cy-1)*100+5)}; }

// Tiny A* (4-neigh) used to draw planned routes.
static vector<P> astar(P s,P g){
  auto H=[&](P a){return abs(a.x-g.x)+abs(a.y-g.y);};
  auto K=[&](P a){return a.y*COLS+a.x;};
  struct N{int f;P p;};
  auto cmp=[](const N&a,const N&b){return a.f>b.f;};
  vector<int> gscore(COLS*ROWS,1e9), parent(COLS*ROWS,-1);
  std::priority_queue<N,vector<N>,decltype(cmp)> pq(cmp);
  auto push=[&](P u,P v){
    if(v.x<0||v.x>=COLS||v.y<0||v.y>=ROWS) return;
    int nk=gscore[K(u)]+1; int kv=K(v);
    if(nk<gscore[kv]){ gscore[kv]=nk; parent[kv]=K(u); pq.push({nk+H(v),v}); }
  };
  gscore[K(s)]=0; pq.push({H(s),s});
  while(!pq.empty()){ P u=pq.top().p; pq.pop(); if(u==g) break;
    push(u,{u.x+1,u.y}); push(u,{u.x-1,u.y}); push(u,{u.x,u.y+1}); push(u,{u.x,u.y-1});
  }
  vector<P> path; if(!(s==g) && parent[K(g)]==-1) return path;
  for(P cur=g;;){ path.push_back(cur); if(cur==s) break; int k=parent[K(cur)]; cur={k%COLS,k/COLS}; }
  reverse(path.begin(),path.end()); return path;
}

// Draw a 5-point star (robot marker).
static void drawStar(sf::RenderTarget& rt, sf::Vector2f c, float r, sf::Color col){
  sf::ConvexShape star; star.setPointCount(10);
  for(int i=0;i<10;i++){
    float ang = -90.f + i*36.f;
    float rad = (i%2==0)? r : r*0.45f;
    float x = c.x + rad * cos(ang*3.1415926f/180.f);
    float y = c.y + rad * sin(ang*3.1415926f/180.f);
    star.setPoint(i,{x,y});
  }
  star.setFillColor(col); star.setOutlineColor(sf::Color::Black); star.setOutlineThickness(2.f);
  rt.draw(star);
}

// Simple world state (just enough for the viz).
struct State{
  P pos = PT.at("nest_y");   // start at Nest Y
  int carried = 0;           // carried crates
  int pool    = 8;           // available at Collect
  int thermo  = 3;           // 0..10, demo only
};

// Build visual “branches”: one per choice (color-coded).
struct Branch{ string label; sf::Color col; vector<P> path; };
static vector<Branch> build_branches(const State& s){
  vector<Branch> B;
  // Move 1 step (up/down/left/right) to show local choices
  auto addStep=[&](const char* name,int dx,int dy,sf::Color c){
    P t={s.pos.x+dx,s.pos.y+dy};
    if(t.x>=0&&t.x<COLS&&t.y>=0&&t.y<ROWS)
      B.push_back({name,c,{s.pos,t}});
  };
  addStep("UP",0,1,  sf::Color(33,150,243));
  addStep("DOWN",0,-1,sf::Color(33,150,243));
  addStep("LEFT",-1,0,sf::Color(33,150,243));
  addStep("RIGHT",1,0,sf::Color(33,150,243));

  // Plan to Collect if we can carry more
  if(s.carried<MAX_CARRY && s.pool>0){
    auto p = astar(s.pos, PT.at("collection"));
    if(!p.empty()) B.push_back({"Plan: Collect", sf::Color(76,175,80), p});
  }
  // Plan to Deliver (Nest Y) if carrying
  if(s.carried>0){
    auto p = astar(s.pos, PT.at("nest_y"));
    if(!p.empty()) B.push_back({"Plan: Deliver (Nest Y)", sf::Color(255,193,7), p});
  }
  // Optional: move to Thermo if idle
  if(s.carried==0 && (s.thermo!=5)){
    auto p = astar(s.pos, PT.at("thermo"));
    if(!p.empty()) B.push_back({"Plan: Thermo", sf::Color(244,67,54), p});
  }
  return B;
}

int main(){
  auto bgp=std::filesystem::current_path()/ "table_bis.png";
  auto fnt=std::filesystem::current_path()/ "DejaVuSans.ttf";
  if(!std::filesystem::exists(bgp)||!std::filesystem::exists(fnt)){ printf("Missing assets.\n"); return 1; }

  const int LEFT_W=COLS*100, WIN_W=LEFT_W+420, WIN_H=ROWS*100+10;
  sf::RenderWindow win(sf::VideoMode(WIN_W,WIN_H),"Eurobot – branching viz"); win.setFramerateLimit(30);
  sf::Texture bg; bg.loadFromFile(bgp.string()); sf::Sprite s_bg(bg); s_bg.setScale((float)LEFT_W/bg.getSize().x,(float)(ROWS*100)/bg.getSize().y);
  sf::Font font; font.loadFromFile(fnt.string());

  State st; int tick=0;
  while(win.isOpen()){
    sf::Event e; while(win.pollEvent(e)) if(e.type==sf::Event::Closed) win.close();

    // Recompute branches at ~10 Hz
    static vector<Branch> branches; if(tick++%3==0) branches = build_branches(st);

    // Demo movement: every ~0.7s follow the longest branch if any
    if(tick%20==0 && !branches.empty()){
      size_t best=0; for(size_t i=1;i<branches.size();++i)
        if(branches[i].path.size()>branches[best].path.size()) best=i;
      if(branches[best].path.size()>1){
        st.pos = branches[best].path[1];
        if(st.pos==PT.at("collection") && st.pool>0 && st.carried<MAX_CARRY){ st.pool--; st.carried++; }
        if(st.pos==PT.at("nest_y") && st.carried>0){ st.carried=0; }
        if(st.pos==PT.at("thermo") && st.thermo!=5){ if(st.thermo<5) st.thermo++; else st.thermo--; }
      }
    }

    // ----- draw
    win.clear(sf::Color::White); s_bg.setPosition(0,0); win.draw(s_bg);

    // grid
    for(int x=0;x<=COLS;x++){ sf::Vertex L[]={{{x*100.f,0.f},{17,17,17}},{{x*100.f,(float)ROWS*100},{17,17,17}}}; win.draw(L,2,sf::Lines); }
    for(int y=0;y<=ROWS;y++){ sf::Vertex L[]={{{0.f,y*100.f},{17,17,17}},{{(float)COLS*100,y*100.f},{17,17,17}}}; win.draw(L,2,sf::Lines); }

    auto box=[&](P p,sf::Color c,const string&txt){
      sf::RectangleShape r({90,90}); r.setPosition(px(p.x,p.y)); r.setFillColor(c);
      r.setOutlineColor({17,17,17}); r.setOutlineThickness(2); win.draw(r);
      sf::Text t(txt,font,16); t.setFillColor(sf::Color::White); t.setPosition(p.x*100.f+20,(ROWS-p.y-1)*100.f+30); win.draw(t);
    };
    box(PT.at("nest_y"),{255,215,0},"Nest Y"); box(PT.at("nest_b"),{30,144,255},"Nest B");
    box(PT.at("collection"),{50,205,50},"Collect"); box(PT.at("pantry"),{139,69,19},"Pantry");
    box(PT.at("thermo"),{220,20,60},"Thermo");

    // draw branches (arrows)
    for(const auto& b: branches){
      for(size_t i=0;i+1<b.path.size();++i){
        auto a=b.path[i], c=b.path[i+1];
        sf::Vector2f A{a.x*100.f+50,(ROWS-a.y-1)*100.f+50}, C{c.x*100.f+50,(ROWS-c.y-1)*100.f+50};
        sf::Vertex seg[]={{A,b.col},{C,b.col}}; win.draw(seg,2,sf::Lines);
        // tiny arrow head
        sf::CircleShape head(5); head.setFillColor(b.col); head.setPosition(C.x-5,C.y-5); win.draw(head);
      }
    }

    // robot as yellow star (easy to spot)
    drawStar(win, {st.pos.x*100.f+50,(ROWS-st.pos.y-1)*100.f+50}, 26.f, sf::Color(255,215,0));

    // right panel
    float bx=LEFT_W+18, by=18;
    sf::Text title("Branching options (visual)",font,20); title.setPosition(bx,by); win.draw(title);
    sf::Text s1("Robot: yellow star at Nest Y",font,16); s1.setPosition(bx,by+34); win.draw(s1);
    sf::Text s2("Arrows = possible routes:\n- Blue: 1-step moves\n- Green: plan to Collect\n- Yellow: plan to Deliver\n- Red: plan to Thermo",font,14);
    s2.setPosition(bx,by+60); win.draw(s2);
    sf::Text stt("Carried: "+to_string(st.carried)+" / "+to_string(MAX_CARRY)+"   Pool: "+to_string(st.pool)+
                 "   Thermo: "+to_string(st.thermo)+"->5",font,16);
    stt.setPosition(bx,WIN_H-40); win.draw(stt);

    win.display();
  }
}
