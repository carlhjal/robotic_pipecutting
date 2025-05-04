// #include <reach/types.h>
// #include <reach/utils.h>
#include <boost/filesystem.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>

using ReachResult = std::vector<ReachRecord, Eigen::aligned_allocator<ReachRecord>>;
using VectorReachResult = std::vector<ReachResult, Eigen::aligned_allocator<ReachResult>>;

static const std::string db_name = "reach.db.xml";

class ReachDatabase
{
public:
  /** @brief Results by reach study iteration */
  VectorReachResult results;

  bool operator==(const ReachDatabase& rhs) const;
  ReachResultSummary calculateResults() const;
  Eigen::MatrixX3f computeHeatMapColors(bool use_full_color_range = false, float hue_low_score = 270.0f,
                                        float hue_high_score = 0.0f) const;

private:
  friend class boost::serialization::access;

  template <class Archive>
  inline void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_NVP(results);
  }
};

void save(const ReachDatabase& db, const std::string& filename);

ReachDatabase load(const std::string& filename);

}  // namespace reach


class ReachDataBaseNode : public rclcpp::Node {
public:
    ReachDataBaseNode() : Node("reach_database_node") {
        boost::filesystem::path db_path = "~/thesis/thesis_ws/src/robotic_pipecutting/reach_utilizer/results/reach.db.xml";    
        reach::ReachDatabase db = reach::load(db_path.string());
        // db.
        // reach::ReachResult res = db.results.back();
        
        // reach::ReachDatabase db;
        // if (!db.load("path/to/reach_database.xml")) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to load reach database.");
        //     return;
    }
private:
    geometry_msgs::msgs::Geometry::msg::PoseArray poses_;
};

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

int main(int argc, char * argv[]) {
    boost::filesystem::path db_path = "~/thesis/thesis_ws/src/robotic_pipecutting/reach_utilizer/results/reach.db.xml";    
    reach::ReachDatabase db = reach::load(db_path.string());
    reach::ReachResult res = db.results.back();
    res[0];

    // reach::ReachResultSummary summary = db.calculateResults();
    // db_ = reach::load(filename);
    // display_->showEnvironment();
}
