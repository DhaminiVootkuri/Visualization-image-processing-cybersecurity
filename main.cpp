// main.cpp
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor2D.h>
#include <vtkLabeledDataMapper.h>
#include <vtkWindowToImageFilter.h>
#include <vtkJPEGWriter.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>
#include <vtkStringArray.h>
#include <vtkPointData.h>
#include <vtkNamedColors.h>
#include <vtkTextProperty.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <filesystem>
#include <cmath>

using Id = std::string;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

static std::vector<std::string> split_any(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : line) {
        if (c == ',' || c == '\t') {
            out.push_back(trim(cur));
            cur.clear();
        }
        else cur.push_back(c);
    }
    out.push_back(trim(cur));
    return out;
}

bool readEntities(const std::string& path, std::unordered_map<Id, std::string>& idToName) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;
    std::string line;
    int ln = 0;
    while (std::getline(ifs, line)) {
        ++ln;
        if (line.empty()) continue;
        if (ln <= 2) continue; 
        auto cols = split_any(line);
        if (cols.size() < 2) continue;
        idToName[cols[0]] = cols[1];
    }
    return true;
}

bool readLinks(const std::string& path, std::vector<std::pair<Id, Id>>& edges) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;
    std::string line;
    int ln = 0;
    while (std::getline(ifs, line)) {
        ++ln;
        if (line.empty()) continue;
        if (ln <= 2) continue;
        auto cols = split_any(line);
        if (cols.size() < 2) continue;
        edges.emplace_back(cols[0], cols[1]);
    }
    return true;
}

bool readPeopleCities(const std::string& path, std::unordered_map<Id, std::string>& idToCity) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;
    std::string line;
    int ln = 0;
    while (std::getline(ifs, line)) {
        ++ln;
        if (line.empty()) continue;
        if (ln <= 2) continue;
        auto cols = split_any(line);
        if (cols.size() < 2) continue;
        idToCity[cols[0]] = cols[1];
    }
    return true;
}

// function to pick employee, handlers, middlemen, leader
void analyzeNetwork(
    const std::vector<std::pair<Id, Id>>& edges,
    const std::unordered_map<Id, std::string>& idToName,
    Id& outEmployee,
    std::vector<Id>& outHandlers,
    std::vector<Id>& outMiddlemen,
    Id& outLeader,
    std::string& outClassification)
{
    // build neighbors
    std::unordered_map<Id, std::unordered_set<Id>> neighbors;
    std::unordered_set<Id> nodes;
    for (auto& e : edges) {
        Id a = e.first; Id b = e.second;
        if (a == b) continue;
        neighbors[a].insert(b);
        neighbors[b].insert(a);
        nodes.insert(a); nodes.insert(b);
    }

    // degree list
    std::vector<std::pair<Id, int>> degs;
    for (auto& n : nodes) degs.emplace_back(n, (int)neighbors[n].size());
    std::sort(degs.begin(), degs.end(), [](auto& a, auto& b) { return a.second > b.second; });

    std::cout << "Top 20 nodes by degree:\n";
    for (size_t i = 0;i < std::min<size_t>(20, degs.size());++i) {
        auto& p = degs[i];
        std::cout << i + 1 << ") id=" << p.first << " deg=" << p.second;
        if (idToName.count(p.first)) std::cout << " name=" << idToName.at(p.first);
        std::cout << "\n";
    }

    // candidate employees
    std::vector<Id> candidates;
    for (auto& p : degs) {
        if (p.second >= 30 && p.second <= 60) candidates.push_back(p.first);
    }
    if (candidates.empty() && !degs.empty()) candidates.push_back(degs[0].first);

    outEmployee = candidates.empty() ? std::string("") : candidates.front();

    // pick handlers among neighbors of employee
    outHandlers.clear();
    if (!outEmployee.empty()) {
        std::vector<std::pair<Id, int>> handlerCands;
        for (auto& nbr : neighbors[outEmployee]) {
            int d = (int)neighbors[nbr].size();
            if (d >= 25 && d <= 60) handlerCands.emplace_back(nbr, d);
        }
        if (handlerCands.size() < 3) {
            handlerCands.clear();
            for (auto& nbr : neighbors[outEmployee]) {
                int d = (int)neighbors[nbr].size();
                if (d >= 15 && d <= 80) handlerCands.emplace_back(nbr, d);
            }
        }
        std::sort(handlerCands.begin(), handlerCands.end(), [](auto& a, auto& b) {return a.second > b.second;});
        for (size_t i = 0;i < std::min<size_t>(3, handlerCands.size()); ++i) outHandlers.push_back(handlerCands[i].first);
    }

    outMiddlemen.clear();
    outLeader.clear();
    outClassification = "unknown";

    if (outHandlers.size() == 3) {
        // look for common middleman
        auto A = neighbors[outHandlers[0]];
        auto B = neighbors[outHandlers[1]];
        auto C = neighbors[outHandlers[2]];
        A.erase(outEmployee); B.erase(outEmployee); C.erase(outEmployee);
        std::vector<Id> common;
        for (auto& x : A) if (B.count(x) && C.count(x)) common.push_back(x);
        if (!common.empty()) {
            outClassification = "A";
            outMiddlemen.push_back(common.front());
            // find leader linked to that middleman
            for (auto& nbr : neighbors[common.front()]) {
                if (nbr == outHandlers[0] || nbr == outHandlers[1] || nbr == outHandlers[2]) continue;
                if ((int)neighbors[nbr].size() >= 80) { outLeader = nbr; break; }
            }
            if (!outLeader.empty()) outClassification = "A (with leader)";
        }
        else {
            // pattern B
            outClassification = "B (tentative)";
            for (auto& h : outHandlers) {
                Id best = ""; int bestDeg = 999999;
                for (auto& n : neighbors[h]) {
                    if (n == outEmployee) continue;
                    int d = (int)neighbors[n].size();
                    if (d < bestDeg) { bestDeg = d; best = n; }
                }
                if (!best.empty()) outMiddlemen.push_back(best);
            }
            // leader detection across middlemen
            std::unordered_map<Id, int> leaderCount;
            for (auto& m : outMiddlemen) {
                for (auto& nbr : neighbors[m]) {
                    if (nbr == outEmployee) continue;
                    if ((int)neighbors[nbr].size() >= 80) leaderCount[nbr]++;
                }
            }
            int maxc = 0;
            for (auto& p : leaderCount) if (p.second > maxc) { maxc = p.second; outLeader = p.first; }
            if (maxc > 0) outClassification = "B (with leader)";
        }
    } 

    // display fallback
    std::cout << "\nFINAL SUMMARY:\n";
    std::cout << "Employee: " << outEmployee << (idToName.count(outEmployee) ? (" (" + idToName.at(outEmployee) + ")") : "") << "\n";
    std::cout << "Handlers: ";
    for (auto& h : outHandlers) std::cout << h << " ";
    std::cout << "\nMiddlemen: ";
    for (auto& m : outMiddlemen) std::cout << m << " ";
    std::cout << "\nFearless Leader: " << (outLeader.empty() ? std::string("none") : outLeader) << (idToName.count(outLeader) ? (" (" + idToName.at(outLeader) + ")") : "") << "\n";
    std::cout << "Classification: " << outClassification << "\n";
}

vtkSmartPointer<vtkPolyData> buildKeyPolyData(
    const std::vector<Id>& nodes,
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<std::array<unsigned char, 3>>& colors,
    const std::vector<std::string>& labels)
{
    auto pts = vtkSmartPointer<vtkPoints>::New();
    auto verts = vtkSmartPointer<vtkCellArray>::New();
    auto lines = vtkSmartPointer<vtkCellArray>::New();

    int n = (int)nodes.size();
    // Count numbers
    int numHandlers = 0, numMiddle = 0;
    bool hasLeader = false;
    for (auto& lab : labels) {
        if (lab.rfind("Handler", 0) == 0) ++numHandlers;
        else if (lab.rfind("Middleman", 0) == 0) ++numMiddle;
        else if (lab.rfind("Leader", 0) == 0) hasLeader = true;
    }

    // compute positions
    std::vector<std::array<double, 3>> pos(n);
    int idxEmployee = -1;
    std::vector<int> handlerIdx, middleIdx; int leaderIdx = -1;
    for (int i = 0;i < n;++i) {
        if (labels[i].rfind("Employee", 0) == 0) idxEmployee = i;
        else if (labels[i].rfind("Handler", 0) == 0) handlerIdx.push_back(i);
        else if (labels[i].rfind("Middleman", 0) == 0) middleIdx.push_back(i);
        else if (labels[i].rfind("Leader", 0) == 0) leaderIdx = i;
    }

    // center params
    const double centerX = 0.0, centerY = 0.0;
    const double handlerR = 250.0;
    const double middleR = 420.0;
    const double leaderY = 650.0;

    // place employee
    if (idxEmployee >= 0) pos[idxEmployee] = { centerX, centerY, 0.0 };
    // place handlers in small circle
    int hcount = (int)handlerIdx.size();
    for (int i = 0;i < hcount;i++) {
        double ang = 2.0 * M_PI * (double)i / (double)std::max(1, hcount);
        pos[handlerIdx[i]] = { centerX + handlerR * cos(ang), centerY + handlerR * sin(ang), 0.0 };
    }
    // place middlemen in larger circle
    int mcount = (int)middleIdx.size();
    for (int i = 0;i < mcount;i++) {
        double ang = 2.0 * M_PI * (double)i / (double)std::max(1, mcount);
        pos[middleIdx[i]] = { centerX + middleR * cos(ang), centerY + middleR * sin(ang), 0.0 };
    }
    // leader
    if (leaderIdx >= 0) pos[leaderIdx] = { centerX, centerY + leaderY, 0.0 };

    // Now create points and verts
    for (int i = 0;i < n;++i) {
        vtkIdType pid = pts->InsertNextPoint(pos[i][0], pos[i][1], pos[i][2]);
        // create a vertex cell for labeling / glyph usage
        verts->InsertNextCell(1);
        verts->InsertCellPoint(pid);
    }

    // lines
    for (auto& e : edges) {
        vtkIdType a = e.first;
        vtkIdType b = e.second;
        lines->InsertNextCell(2);
        lines->InsertCellPoint(a);
        lines->InsertCellPoint(b);
    }

    auto poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(pts);
    poly->SetVerts(verts);
    poly->SetLines(lines);

    // add color array
    auto colArr = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colArr->SetNumberOfComponents(3);
    colArr->SetName("node_color");
    for (int i = 0;i < n;++i) {
        unsigned char tuple[3] = { 0,0,0 };
        if ((size_t)i < colors.size()) {
            tuple[0] = colors[i][0];
            tuple[1] = colors[i][1];
            tuple[2] = colors[i][2];
        }
        colArr->InsertNextTypedTuple(tuple);
    }
    poly->GetPointData()->AddArray(colArr);

    // add label string array
    auto sarr = vtkSmartPointer<vtkStringArray>::New();
    sarr->SetName("label");
    for (int i = 0;i < n;++i) sarr->InsertNextValue(labels[i]);
    poly->GetPointData()->AddArray(sarr);

    return poly;
}

int main(int argc, char** argv) {
    // read dataset as provided
    const std::string baseFolder = "D:/Smile_Final/FinalProject/M2-Social Net and Geo";
    std::filesystem::path entPath = std::filesystem::path(baseFolder) / "Entities_Table.txt";
    std::filesystem::path linksPath = std::filesystem::path(baseFolder) / "Links_Table.txt";
    std::filesystem::path citiesPath = std::filesystem::path(baseFolder) / "People-Cities.txt";

    if (argc >= 4) {
        entPath = argv[1];
        linksPath = argv[2];
        citiesPath = argv[3];
    }

    if (!std::filesystem::exists(entPath) || !std::filesystem::exists(linksPath)) {
        std::cerr << "ERROR: Required M2 files not found. Provide paths or place the dataset in the project folder.\n";
        return EXIT_FAILURE;
    }

    std::cout << "Using:\n  " << entPath.string() << "\n  " << linksPath.string() << "\n  Cities: " << (std::filesystem::exists(citiesPath) ? citiesPath.string() : std::string("NOT FOUND")) << "\n\n";

    // parse input
    std::unordered_map<Id, std::string> idToName;
    if (!readEntities(entPath.string(), idToName)) {
        std::cerr << "ERROR reading entities file\n";
        return EXIT_FAILURE;
    }
    std::vector<std::pair<Id, Id>> edges;
    if (!readLinks(linksPath.string(), edges)) {
        std::cerr << "ERROR reading links file\n";
        return EXIT_FAILURE;
    }
    std::unordered_map<Id, std::string> idToCity;
    if (std::filesystem::exists(citiesPath)) {
        if (!readPeopleCities(citiesPath.string(), idToCity)) {
            std::cout << "Warning: People-Cities read failed; proceeding without city positions.\n";
            idToCity.clear();
        }
    }

    // analyze and get key nodes
    Id employee;
    std::vector<Id> handlers;
    std::vector<Id> middlemen;
    Id leader;
    std::string classification;
    analyzeNetwork(edges, idToName, employee, handlers, middlemen, leader, classification);

    if (employee.empty()) {
        std::cerr << "No employee candidate found - aborting.\n";
        return EXIT_FAILURE;
    }

    //now build small node list for key-hierarchy visualization
    std::vector<Id> keyNodes;
    keyNodes.push_back(employee);
    for (auto& h : handlers) keyNodes.push_back(h);
    for (auto& m : middlemen) keyNodes.push_back(m);
    if (!leader.empty()) keyNodes.push_back(leader);

    // map id->index
    std::unordered_map<Id, int> idx;
    for (int i = 0;i < (int)keyNodes.size();++i) idx[keyNodes[i]] = i;

    // build edges among the key nodes
    std::vector<std::pair<int, int>> keyEdges;
    for (auto& h : handlers) if (idx.count(h)) {
        keyEdges.emplace_back(idx[employee], idx[h]);
    }
    // connect handlers 
    if (!middlemen.empty()) {
        // naive: for each handler connect to its associated middleman if both present in idx
        for (size_t i = 0;i < handlers.size();++i) {
            if (i < middlemen.size()) {
                if (idx.count(handlers[i]) && idx.count(middlemen[i])) keyEdges.emplace_back(idx[handlers[i]], idx[middlemen[i]]);
            }
            else {
                // fallback: connect handler to first middleman if exists
                if (idx.count(handlers[i]) && idx.count(middlemen[0])) keyEdges.emplace_back(idx[handlers[i]], idx[middlemen[0]]);
            }
        }
    }
    // connect each middleman to leader
    if (!leader.empty()) {
        for (auto& m : middlemen) if (idx.count(m) && idx.count(leader)) keyEdges.emplace_back(idx[m], idx[leader]);
    }

    // prepare per-node colors and labels
    std::vector<std::array<unsigned char, 3>> nodeColors;
    std::vector<std::string> labels;
    for (int i = 0;i < (int)keyNodes.size();++i) {
        Id id = keyNodes[i];
        std::string name = idToName.count(id) ? idToName[id] : id;
        // determine role
        std::string role = "Other";
        if (i == 0) role = "Employee";
        else {
            bool isHandler = false, isMiddle = false, isLeader = false;
            for (auto& h : handlers) if (h == id) isHandler = true;
            for (auto& m : middlemen) if (m == id) isMiddle = true;
            if (!leader.empty() && id == leader) isLeader = true;
            if (isHandler) role = "Handler";
            else if (isMiddle) role = "Middleman";
            else if (isLeader) role = "Leader";
            else role = "Other";
        }
        
        std::array<unsigned char, 3> col = { 160,160,160 };
        if (role == "Employee") col = { 56,176,40 };
        else if (role == "Handler") col = { 38,115,200 };
        else if (role == "Middleman") col = { 220,120,30 };
        else if (role == "Leader") col = { 200,30,30 };
        nodeColors.push_back(col);
        
        std::string lab = role + "\n" + name;
        labels.push_back(lab);
    }

    // build polydata
    auto keyPoly = buildKeyPolyData(keyNodes, keyEdges, nodeColors, labels);

    // create separate polydata for edges only
    auto edgePoly = vtkSmartPointer<vtkPolyData>::New();
    edgePoly->SetPoints(keyPoly->GetPoints());
    edgePoly->SetLines(keyPoly->GetLines());

    // Node mapper 
    auto nodeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    nodeMapper->SetInputData(keyPoly);
    nodeMapper->SetScalarModeToUsePointFieldData();
    nodeMapper->SelectColorArray("node_color");
    nodeMapper->ScalarVisibilityOn();

    // Edge mapper
    auto edgeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    edgeMapper->SetInputData(edgePoly);
    
    edgeMapper->ScalarVisibilityOff();

    // Actors
    auto nodeActor = vtkSmartPointer<vtkActor>::New();
    nodeActor->SetMapper(nodeMapper);
    nodeActor->GetProperty()->SetPointSize(14); 

    nodeActor->GetProperty()->SetRenderPointsAsSpheres(false);
  
    auto edgeActor = vtkSmartPointer<vtkActor>::New();
    edgeActor->SetMapper(edgeMapper);
    edgeActor->GetProperty()->SetColor(0.75, 0.75, 0.75);
    edgeActor->GetProperty()->SetLineWidth(2.0);

    auto labelMapper = vtkSmartPointer<vtkLabeledDataMapper>::New();
    labelMapper->SetInputData(keyPoly);
    labelMapper->SetLabelModeToLabelFieldData();
    labelMapper->SetFieldDataName("label");
    labelMapper->GetLabelTextProperty()->SetFontSize(16);
    labelMapper->GetLabelTextProperty()->BoldOn();
    labelMapper->GetLabelTextProperty()->SetColor(1.0, 1.0, 1.0);

    auto labelActor = vtkSmartPointer<vtkActor2D>::New();
    labelActor->SetMapper(labelMapper);

    // Renderer + window + interactor
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(0.12, 0.12, 0.12);

    auto renWin = vtkSmartPointer<vtkRenderWindow>::New();
    renWin->AddRenderer(renderer);
    renWin->SetSize(1400, 900);

    auto iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    iren->SetRenderWindow(renWin);

    // Add actors 
    renderer->AddActor(edgeActor);
    renderer->AddActor(nodeActor);
    renderer->AddActor(labelActor);

    renderer->ResetCamera();
    renWin->Render();

    // My output directory
    std::string outDir = "D:/Smile_Final/FinalProject/output";
    if (!std::filesystem::exists(outDir)) std::filesystem::create_directories(outDir);

    // Save image 
    auto w2i = vtkSmartPointer<vtkWindowToImageFilter>::New();
    w2i->SetInput(renWin);
    w2i->Update();

    auto jpg = vtkSmartPointer<vtkJPEGWriter>::New();
    std::string jpgName = outDir + "/keyHierarchy.jpg";
    jpg->SetFileName(jpgName.c_str());
    jpg->SetInputConnection(w2i->GetOutputPort());
    jpg->Write();

    std::cout << "keyHierarchy image saved in: " << outDir << "\n";

    // Start interactive session
    iren->Start();

    return EXIT_SUCCESS;
}
