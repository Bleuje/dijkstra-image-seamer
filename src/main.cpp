// Translation from Qt C++ to openFrameworks C++ with ChatGPT o1,
// of original code by Etienne Jacob.
// Comments from ChatGPT, the original code was uncommented, it's there: https://gist.github.com/Bleuje/91c22639d0f312b38a07d19ee032739a

#include "ofMain.h"       // for ofImage, ofColor, etc.
#include <bits/stdc++.h>  // or include the specific STL headers you need
#include <chrono>         // for timing

using namespace std;

/*
==============================================================================
 ALGORITHM OVERVIEW
 
 This code stitches two overlapping parts of the same image (or repeated image)
 by finding a minimal-difference seam. The steps are as follows:

 1) FIND OFFSET
    - The function `findOffset` scans different overlap widths (offsets) 
      to see which produces the lowest average color difference between 
      corresponding overlap pixels.
    - The offset that yields the minimal average difference is chosen.

 2) DEFINE COST MATRIX
    - Once the best offset is found, `defineCosts` creates a 2D cost matrix 
      covering the overlap region. Each cell (y, x) represents the squared 
      color difference between the overlapping pixels of the original image.

 3) DIJKSTRA FOR MINIMAL SEAM
    - We treat the cost matrix like a grid and run Dijkstra's algorithm 
      (`dijkstraWithPQ`) to find a path from the top row to the bottom row 
      that has the minimal total cost (sum of color differences).
    - This path indicates where we should "cut" between the two images 
      so that the seam runs through areas of minimal difference.

 4) SEAM MAPPING & FLOOD FILL
    - After finding the seam path, we mark it in a "mask" array as a special 
      label (e.g., label=3). We then flood-fill from the left edge (label=1) 
      and the right edge (label=2). This way, each pixel is labeled either as 
      belonging to the left side, right side, or on the seam.

 5) MERGING THE FINAL IMAGE
    - Using the mask, each output pixel is assigned from either the left image 
      region or the right image region, avoiding double coverage of the overlap.
    - Pixels on the seam can be colored red (if `SHOW_SEAM` is true), or 
      averaged between the left and right to smooth the boundary.

 6) HORIZONTAL VS. VERTICAL
    - For a horizontal stitch, we run the algorithm directly.
    - For a vertical stitch, we rotate the image 90 degrees, run the same 
      horizontal logic, then unrotate the result.

 7) TIMING
    - We optionally measure and print elapsed time (in milliseconds) to see 
      how long the stitching process takes.

 In essence, this method finds the best overlap offset, computes a minimal-
 difference path (using Dijkstra on a cost matrix of color differences), 
 and merges the two images along that path for a visually seamless stitch.
==============================================================================
*/

// We skip testing extremely small overlaps (less than 11 pixels), 
// since those might be too trivial or noisy to provide a good seam.
const int START_WINDOW_SIZE = 11;

// When searching for the best offset, we move in steps of 4 instead of 1, 
// trading some accuracy for speed. (You can set this to 1 if you want 
// a more thorough search but can handle the extra computation time.)
const int FIND_OFFSET_STEP  = 4;

bool SHOW_SEAM = false;
//--------------------------------------------------------------------

// Move directions for dijkstra / flood fill
pair<int,int> dpos[4] = {{1,0},{-1,0},{0,1},{0,-1}};

//--------------------------------------------------------------------
// Function: ofImageToVector
// Purpose:  Convert an ofImage into a 2D vector of ofColor.
//           This allows easier manipulation (like in the Qt version).
//--------------------------------------------------------------------
vector<vector<ofColor>> ofImageToVector(const ofImage &img){
    int w = img.getWidth();
    int h = img.getHeight();

    vector<vector<ofColor>> res(h, vector<ofColor>(w));
    for(int y=0; y<h; y++){
        for(int x=0; x<w; x++){
            // Extract the color at pixel (x, y)
            res[y][x] = img.getColor(x,y);
        }
    }
    return res;
}

//--------------------------------------------------------------------
// Function: vectorToOfImage
// Purpose:  Convert a 2D vector of ofColor back into an ofImage.
//--------------------------------------------------------------------
ofImage vectorToOfImage(const vector<vector<ofColor>> &vec){
    int h = vec.size();
    int w = (h > 0) ? vec[0].size() : 0;

    // Allocate an ofImage (with or without alpha as needed)
    ofImage out;
    out.allocate(w, h, OF_IMAGE_COLOR_ALPHA); 

    // Copy color data from 2D vector into the ofImage's pixels
    for(int y=0; y<h; y++){
        for(int x=0; x<w; x++){
            out.setColor(x, y, vec[y][x]);
        }
    }
    out.update(); // Make sure pixels are updated
    return out;
}

//--------------------------------------------------------------------
// Function: rotate
// Purpose:  Rotate the input 2D array of colors 90 degrees clockwise.
//--------------------------------------------------------------------
vector<vector<ofColor>> rotate(const vector<vector<ofColor>> &vec){
    int h = vec.size();
    int w = vec[0].size();

    // Rotated image has dimensions (w x h)
    vector<vector<ofColor>> res(w, vector<ofColor>(h));
    for(int y=0; y<h; y++){
        for(int x=0; x<w; x++){
            // 90 deg clockwise transformation: (x, y) -> (w-1-x, y)
            res[w - 1 - x][y] = vec[y][x];
        }
    }
    return res;
}

//--------------------------------------------------------------------
// Function: unrotate
// Purpose:  The inverse of rotate (rotate 3 times to get original).
//           This "unrotates" the image (i.e., rotates 270 deg clockwise,
//           which is the same as 90 deg counterclockwise).
//--------------------------------------------------------------------
vector<vector<ofColor>> unrotate(const vector<vector<ofColor>> &vec){
    // rotate 3 times
    return rotate(rotate(rotate(vec)));
}

//--------------------------------------------------------------------
// Function: colorDifference
// Purpose:  Compute the squared distance between two ofColor's RGB values.
//           This is used as a cost measure for seam matching.
//--------------------------------------------------------------------
int colorDifference(const ofColor &c1, const ofColor &c2){
    int dr = (int)c1.r - (int)c2.r;
    int dg = (int)c1.g - (int)c2.g;
    int db = (int)c1.b - (int)c2.b;
    return dr*dr + dg*dg + db*db;
}

//--------------------------------------------------------------------
// Function: findOffset
// Purpose:  Determines how far to shift one side of the image horizontally
//           so that the overlapping region has minimal difference.
//           This tries different 'k' offsets to see which yields the
//           lowest average colorDifference over the overlap.
//--------------------------------------------------------------------
int findOffset(const vector<vector<ofColor>> &vec){
    int w = vec[0].size();
    int h = vec.size();

    double best = 1e18; // track the best (lowest) difference
    int res = -1;

    // We only search offsets >= START_WINDOW_SIZE and < w/2
    for(int k = max(START_WINDOW_SIZE, 20); k < w/2; k += FIND_OFFSET_STEP){
        long long sum = 0;
        // For each row, compare the overlap area
        for(int y=0; y<h; y++){
            for(int x=0; x<k; x++){
                // left side pixel is vec[y][x], right side pixel is vec[y][w-k + x]
                sum += colorDifference(vec[y][w - k + x], vec[y][x]);
            }
        }
        double average = (double)sum / (double)(h * k);

        // Keep track of the best offset
        if(average < best){
            best = average;
            res = k;
        }
    }
    return res;
}

//--------------------------------------------------------------------
// Function: defineCosts
// Purpose:  Create a cost matrix where costs[y][x] = colorDifference
//           between overlapping pixels at (x) and (w-k + x) in the image.
//           We will run a path-finding algorithm on this cost matrix.
//--------------------------------------------------------------------
vector<vector<double>> defineCosts(const vector<vector<ofColor>> &vec, int k){
    int w = vec[0].size();
    int h = vec.size();

    vector<vector<double>> costs(h, vector<double>(k, 0.0));
    for(int y=0; y<h; y++){
        for(int x=0; x<k; x++){
            costs[y][x] = (double)colorDifference(vec[y][w - k + x], vec[y][x]);
        }
    }
    return costs;
}

//--------------------------------------------------------------------
// We'll use a typical Dijkstra approach with a priority queue
//--------------------------------------------------------------------
typedef pair<double, pair<int,int>> doublePair;

//--------------------------------------------------------------------
// Function: dijkstraWithPQ
// Purpose:  Find a minimal-cost path from the top row (0, j0) to the
//           bottom row (h-1, any column), where the cost is derived
//           from the cost matrix. This path marks where we will "cut"
//           between the two overlapping images.
//--------------------------------------------------------------------
vector<pair<int,int>> dijkstraWithPQ(const vector<vector<double>> &costs, int j0){
    int h = costs.size();
    int k = costs[0].size();

    // Distances, initialize to large
    vector<vector<double>> dist(h, vector<double>(k, 1e18));
    // Predecessor array to reconstruct path
    vector<vector<pair<int,int>>> prev(h, vector<pair<int,int>>(k, make_pair(-1,-1)));

    // Start position is top row, column j0
    dist[0][j0] = 0.0;

    // Min-heap for (distance, (i, j))
    priority_queue<doublePair, vector<doublePair>, greater<doublePair>> Q;
    Q.push({0.0, {0,j0}});

    while(!Q.empty()){
        // Extract the pair with the minimum distance
        auto [curDist, u] = Q.top();
        Q.pop();

        int i1 = u.first;
        int j1 = u.second;

        // If we've already found a better route, skip
        if(fabs(dist[i1][j1] - curDist) > 1e-12){
            continue;
        }

        // Explore neighbors
        for(int a=0; a<4; a++){
            int vi = i1 + dpos[a].first;
            int vj = j1 + dpos[a].second;
            // Check boundaries
            if(vi >= 0 && vi < h && vj >= 0 && vj < k){
                // Cost to move from (i1, j1) to (vi, vj)
                double alt = dist[i1][j1] + (costs[i1][j1] + costs[vi][vj]);
                // If we found a cheaper path, update
                if(alt < dist[vi][vj]){
                    dist[vi][vj] = alt;
                    prev[vi][vj] = make_pair(i1, j1);
                    Q.push({alt, {vi,vj}});
                }
            }
        }
    }

    // Find minimal distance in bottom row
    double the_min = 1e18;
    int jend = -1;
    for(int j=0; j<k; j++){
        if(dist[h-1][j] < the_min){
            the_min = dist[h-1][j];
            jend = j;
        }
    }

    cout << "THE MIN DISTANCE WITH DIJKSTRA_PQ : " << the_min << endl;

    // Reconstruct path from bottom row up to top
    vector<pair<int,int>> path;
    pair<int,int> end = make_pair(h-1, jend);
    path.push_back(end);

    // Step backwards until we reach (0, j0)
    while(!(end.first == 0 && end.second == j0)){
        end = prev[end.first][end.second];
        path.push_back(end);
    }

    // Currently reversed (bottom->top), so flip it
    reverse(path.begin(), path.end());
    return path;
}

//--------------------------------------------------------------------
// Function: findStart
// Purpose:  Decide which column in the top row we should start the seam
//           (the minimal sum of costs in a small window near the top).
//--------------------------------------------------------------------
int findStart(const vector<vector<double>> &costs){
    int h = costs.size();
    int k = costs[0].size();
    int half = (START_WINDOW_SIZE - 1) / 2;

    double the_min = 1e18;
    int ind = -1;

    // We scan across columns in the top row, 
    // taking a (START_WINDOW_SIZE x START_WINDOW_SIZE) window.
    for(int x = half; x < k - half; x++){
        long long sum = 0;
        for(int yy=0; yy<=half; yy++){
            for(int xx = x-half; xx <= x+half; xx++){
                sum += (long long)costs[yy][xx];
            }
        }
        if(sum < the_min){
            the_min = sum;
            ind = x;
        }
    }
    return ind;
}

//--------------------------------------------------------------------
// Function: floodFill
// Purpose:  Flood-fill an integer mask array, assigning the region
//           to "value".  Used to label which side of the seam is
//           "left" or "right."
//--------------------------------------------------------------------
void floodFill(vector<vector<int>> &res1, int i0, int j0, int value){
    int h = res1.size();
    int w = res1[0].size();

    // We'll fill from (i0, j0) with 'value'
    res1[i0][j0] = value;
    stack<pair<int,int>> st;
    st.push({i0,j0});

    while(!st.empty()){
        auto p = st.top();
        st.pop();
        for(int a=0; a<4; a++){
            int vi = p.first + dpos[a].first;
            int vj = p.second + dpos[a].second;
            // If it's within bounds and not yet filled, fill it
            if(vi>=0 && vi<h && vj>=0 && vj<w && res1[vi][vj] == 0){
                res1[vi][vj] = value;
                st.push({vi,vj});
            }
        }
    }
}

//--------------------------------------------------------------------
// Function: imageFromDijsktra
// Purpose:  Given the original image array 'im', an 'offset', and a 
//           seam path 'path', produce a new combined image (2*w - offset wide)
//           in which the seam divides the left and right overlap areas
//           at points of minimal difference.
//--------------------------------------------------------------------
vector<vector<ofColor>> imageFromDijsktra(const vector<vector<ofColor>> &im,
                                          int offset,
                                          const vector<pair<int,int>> &path)
{
    int h = im.size();
    int w = im[0].size();

    // Our final stitched image has width (2*w - offset)
    vector<vector<ofColor>> out(h, vector<ofColor>(2*w - offset, ofColor(0,0,0,255)));

    // We'll store an integer "mask" to label each pixel:
    //  0 = unfilled
    //  1 = left side or region
    //  2 = right side or region
    //  3 = seam (the dijkstra path)
    vector<vector<int>> mask(h, vector<int>(2*w - offset, 0));

    // Mark the seam path as "3"
    for(const auto &p : path){
        int i = p.first;
        int j = p.second;
        // The seam in the final image is at column (w - offset + j)
        mask[i][w - offset + j] = 3;
    }

    // Fill left side (region "1") by flood fill from the left edge
    floodFill(mask, h/2, 0, 1);

    // Fill right side (region "2") by flood fill from the right edge
    floodFill(mask, h/2, (2*w - offset) - 1, 2);

    // Now fill in the final color
    for(int i=0; i<h; i++){
        for(int j=0; j < (2*w - offset); j++){
            
            // Condition 1: if j < (w-offset) or (j < w && mask is 1)
            // Means it's either strictly in the left portion or assigned to region 1
            if(j < (w - offset) || (j < w && mask[i][j] == 1)){
                out[i][j] = im[i][j];
            }
            // Condition 2: if j >= w or (j >= (w-offset) && mask is 2)
            // Means it's in the right portion or assigned to region 2
            else if(j >= w || (j >= (w - offset) && mask[i][j] == 2)){
                int jj = j - (w - offset); // position in the right half
                out[i][j] = im[i][jj];
            }
            else {
                // Overlapping region not explicitly assigned => on the "border".
                // If SHOW_SEAM is true, highlight in red. Else, average the pixels.
                ofColor c1 = im[i][j];
                int jj = j - (w - offset);
                ofColor c2 = im[i][jj];
                if(SHOW_SEAM){
                    out[i][j] = ofColor(255,0,0,255); // red line
                } else {
                    // average the two
                    unsigned char rr = (c1.r + c2.r)/2;
                    unsigned char gg = (c1.g + c2.g)/2;
                    unsigned char bb = (c1.b + c2.b)/2;
                    out[i][j] = ofColor(rr, gg, bb, 255);
                }
            }
        }
    }

    return out;
}

//--------------------------------------------------------------------
// Function: performAlgoH
// Purpose:  Perform the "horizontal" seam stitching. Find offset,
//           build a cost matrix, run Dijkstra, then produce a 
//           horizontally stitched image that merges both sides
//           at the minimal-difference path.
//--------------------------------------------------------------------
ofImage performAlgoH(const ofImage &img){
    // Convert to 2D array of colors
    auto vec = ofImageToVector(img);

    // Optional debug save
    ofImage testImg = vectorToOfImage(vec);
    testImg.save("testsave2.png");

    // 1) Find the horizontal offset
    int offset = findOffset(vec);
    cout << "Found offset : " << offset << endl;

    // 2) Define the cost matrix for the overlapping region
    auto costs = defineCosts(vec, offset);

    // 3) Find which column to start from in the top row
    int j0 = findStart(costs);
    cout << "j0 : " << j0 << endl;

    // 4) Use Dijkstra to find the minimal difference path
    auto path = dijkstraWithPQ(costs, j0);
    cout << "Dijkstra finished\n";

    // 5) Build the final stitched image based on that seam
    auto joined = imageFromDijsktra(vec, offset, path);
    cout << "Flood Fill done\n";

    // Convert back to ofImage
    ofImage resImg = vectorToOfImage(joined);
    return resImg;
}

//--------------------------------------------------------------------
// Function: performAlgoV
// Purpose:  Perform the "vertical" seam stitching by rotating the image
//           90 deg, reusing the same approach as horizontal, then
//           unrotating the result.
//--------------------------------------------------------------------
ofImage performAlgoV(const ofImage &img){
    // Convert image to 2D array
    auto vec0 = ofImageToVector(img);
    // Rotate 90 deg
    auto vec = rotate(vec0);

    // Optional debug save
    ofImage testImg = vectorToOfImage(vec);
    testImg.save("testsave2.png");

    // 1) Find offset in rotated domain
    int offset = findOffset(vec);
    cout << "Found offset : " << offset << endl;

    // 2) Define cost matrix
    auto costs = defineCosts(vec, offset);

    // 3) Choose a start column in the top row (now rotated)
    int j0 = findStart(costs);
    cout << "j0 : " << j0 << endl;

    // 4) Dijkstra
    auto path = dijkstraWithPQ(costs, j0);
    cout << "Dijkstra finished\n";

    // 5) Rebuild stitched image from path
    auto joined = imageFromDijsktra(vec, offset, path);
    cout << "Flood Fill done\n";

    // 6) Unrotate result to original orientation
    auto unrot = unrotate(joined);

    // Convert to ofImage
    ofImage resImg = vectorToOfImage(unrot);
    return resImg;
}

//--------------------------------------------------------------------
// Main entry point
// This is a simple console-like usage of openFrameworks. It doesn't
// create a window or run an interactive "ofApp". Instead, it loads
// an image, runs the horizontal & vertical seam stitching, saves
// the final result, and prints elapsed time to the console.
//--------------------------------------------------------------------
int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_image> <output_image>" << std::endl;
        return 1;
    }

    std::string inputFilename = argv[1];
    std::string outputFilename = argv[2];

    // Check for the optional "--show-seam" argument
    if (argc > 3 && std::string(argv[3]) == "--show-seam") {
        SHOW_SEAM = true;
    }

    // Attempt to load the original image
    ofImage img;
    bool loaded = img.load(inputFilename);
    if(!loaded){
        cerr << "Could not load image: " << inputFilename << endl;
        return -1;
    }

    // Start timing from this point
    auto startTime = std::chrono::high_resolution_clock::now();

    // Perform horizontal seam join
    ofImage resImg = performAlgoH(img);

    // Then perform vertical seam join
    resImg = performAlgoV(resImg);

    // (Optional) If desired, repeat steps for further refinement
    // resImg = performAlgoV(resImg);
    // resImg = performAlgoH(resImg);

    // Save the final stitched result
    resImg.save(outputFilename);

    // Stop timing 
    auto endTime = std::chrono::high_resolution_clock::now();
    // Compute duration in milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

    // Print summary
    cout << "Done. Saved " << outputFilename << "." << endl;
    cout << "Elapsed time: " << duration << " ms." << endl;

    return 0;
}
