#include <reach/types.h>
#include <boost/filesystem.hpp>
#include <string>

static const std::string db_name = "reach.db.xml";

// void load_db(const boost::filesystem::path& results_folder, reach::ReachDatabase& db, reach::ReachResultSummary& summary) {

//     boost::filesystem::path results_folder = db_path + "/results";
//     boost::filesystem::path db_file;

//     // Search for the .db.xml file
//     for (boost::filesystem::directory_iterator it(results_folder), endit; it != endit; ++it)
//     {
//         if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ".xml")
//         {
//             if (it->path().filename() == OPT_DB_NAME)
//             {
//                 db_file = it->path();
//                 break;
//             }
//         }
//     }
// }

// int main(int argc, char * argv[]) {
//     boost::filesystem::path db_path = "~/thesis/thesis_ws/src/robotic_pipecutting/reach_utilizer/results/reach.db.xml";
//     // reach::ReachDatabase db;
    
//     reach::ReachDatabase db = reach::load(db_path.string());
//     reach::ReachResultSummary summary = db.calculateResults();
// }

int main(int argc, char * argv[]) {
    return 0;
}
// using ReachResult = std::vector<ReachRecord, Eigen::aligned_allocator<ReachRecord>>;
// using VectorReachResult = std::vector<ReachResult, Eigen::aligned_allocator<ReachResult>>;

// ReachResultSummary calculateResults(const ReachResult& db);
