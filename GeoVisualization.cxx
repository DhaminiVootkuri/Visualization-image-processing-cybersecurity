#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkGlyph3DMapper.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>

// Convert latitude and longitude (in degrees) to Cartesian coordinates
void AddPoint(double lon_deg, double lat_deg, double R, vtkPoints* pts) {
    const double d2r = M_PI / 180.0;
    double lon = lon_deg * d2r;
    double lat = lat_deg * d2r;
    double x = R * std::cos(lat) * std::cos(lon);
    double y = R * std::cos(lat) * std::sin(lon);
    double z = R * std::sin(lat);
    pts->InsertNextPoint(x, y, z);
}

// Read sample CSV (longitude, latitude) file
vtkSmartPointer<vtkPoints> ReadPointsFromCSV(const std::string& filename, double R) {
    auto pts = vtkSmartPointer<vtkPoints>::New();
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string lon_str, lat_str;
        if (std::getline(ss, lon_str, ',') && std::getline(ss, lat_str, ',')) {
            double lon = std::stod(lon_str);
            double lat = std::stod(lat_str);
            AddPoint(lon, lat, R, pts);
        }
    }
    return pts;
}

int main() {
    // 1) Create Earth sphere
    auto earth = vtkSmartPointer<vtkSphereSource>::New();
    earth->SetRadius(1.0);
    earth->SetThetaResolution(64);
    earth->SetPhiResolution(64);
    earth->Update();

    // 2) Read example coordinates from CSV
    auto pts = ReadPointsFromCSV("cities.csv", 1.01);

    // 3) Create glyphs for points
    auto pointCloud = vtkSmartPointer<vtkPolyData>::New();
    pointCloud->SetPoints(pts);

    auto marker = vtkSmartPointer<vtkSphereSource>::New();
    marker->SetRadius(0.02);

    auto glyphMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
    glyphMapper->SetInputData(pointCloud);
    glyphMapper->SetSourceConnection(marker->GetOutputPort());
    glyphMapper->SetScalarVisibility(false);

    auto glyphActor = vtkSmartPointer<vtkActor>::New();
    glyphActor->SetMapper(glyphMapper);

    auto earthMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    earthMapper->SetInputConnection(earth->GetOutputPort());

    auto earthActor = vtkSmartPointer<vtkActor>::New();
    earthActor->SetMapper(earthMapper);

    // 4) Set up renderer and window
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(earthActor);
    renderer->AddActor(glyphActor);
    renderer->SetBackground(0.1, 0.1, 0.2);

    auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetWindowName("GeoGlobe - VTK 9.5.2");

    auto interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);

    renderWindow->Render();
    interactor->Start();

    return 0;
}
