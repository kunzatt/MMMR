DROP DATABASE IF EXISTS mmmr;

CREATE SCHEMA IF NOT EXISTS mmmr DEFAULT CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci;

USE mmmr;

CREATE TABLE accounts(
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    email VARCHAR(100) UNIQUE NOT NULL,
    password VARCHAR(100),
    address VARCHAR(255),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    INDEX idx_email (email)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

CREATE TABLE profiles(
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    account_id BIGINT NOT NULL,
    nickname VARCHAR(20) NOT NULL,
    callsign ENUM('미미', '루나', '아리', '노바', '해태', '세라', '코코', '나비') NOT NULL,
    count INT NOT NULL DEFAULT 0,
    deleted TINYINT NOT NULL DEFAULT 0,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    CONSTRAINT fk_profiles_account_id FOREIGN KEY (account_id) REFERENCES accounts(id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

CREATE TABLE schedules (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    profile_id BIGINT NOT NULL,
    title VARCHAR(50) NOT NULL,
    start_date TIMESTAMP NOT NULL,
    end_date TIMESTAMP NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    deleted TINYINT NOT NULL DEFAULT 0,
    CONSTRAINT fk_schedules_profile_id FOREIGN KEY (profile_id) REFERENCES profiles(id) ON DELETE CASCADE,
    INDEX idx_deleted (deleted)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

CREATE TABLE todos (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    profile_id BIGINT NOT NULL,
    content VARCHAR(256) NOT NULL,
    is_done TINYINT NOT NULL DEFAULT 0,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    deleted TINYINT NOT NULL DEFAULT 0,
    CONSTRAINT fk_todos_profile_id FOREIGN KEY (profile_id) REFERENCES profiles(id) ON DELETE CASCADE,
    INDEX idx_is_done (is_done),
    INDEX idx_deleted (deleted) 
)ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

CREATE TABLE metros (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    profile_id BIGINT NOT NULL,
    line INT NOT NULL,
    station VARCHAR(20) NOT NULL,
    direction VARCHAR(20),
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    deleted TINYINT NOT NULL DEFAULT 0,
    CONSTRAINT fk_metros_profile_id FOREIGN KEY (profile_id) REFERENCES profiles(id) ON DELETE CASCADE,
    INDEX idx_deleted (deleted)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

CREATE TABLE buses (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    profile_id BIGINT NOT NULL,
    route_id INT NOT NULL,
    route VARCHAR(10) NOT NULL,
    station_id INT NOT NULL,
    station VARCHAR(20) NOT NULL,
    direction VARCHAR(20),
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    deleted TINYINT NOT NULL DEFAULT 0,
    CONSTRAINT fk_buses_profile_id FOREIGN KEY (profile_id) REFERENCES profiles(id) ON DELETE CASCADE,
    INDEX idx_deleted (deleted)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

CREATE TABLE businformations (
    id INT AUTO_INCREMENT PRIMARY KEY,
    route_id INT NOT NULL,
    route VARCHAR(100) NOT NULL,
    sequence INT NOT NULL,
    station_id INT NOT NULL,
    station VARCHAR(100) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

CREATE TABLE metro_informations (
    id INT AUTO_INCREMENT PRIMARY KEY, 
    line_number VARCHAR(100) NOT NULL, 
    station_name VARCHAR(100) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

CREATE TABLE news (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    title TEXT NOT NULL,
    content TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

CREATE TABLE home_devices (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    account_id BIGINT NOT NULL,
    device VARCHAR(100) NOT NULL,
    turned VARCHAR(10) NOT NULL,
    CONSTRAINT fk_home_devices_account_id FOREIGN KEY (account_id) REFERENCES accounts(id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

-- 트리거 생성
DELIMITER //

-- metros 테이블에 INSERT가 발생할 때 profiles 테이블의 count를 증가시키는 트리거
CREATE TRIGGER after_metro_insert
AFTER INSERT ON metros
FOR EACH ROW
BEGIN
    IF NEW.deleted = 0 THEN
        UPDATE profiles 
        SET count = count + 1 
        WHERE id = NEW.profile_id;
    END IF;
END//

-- buses 테이블에 INSERT가 발생할 때 profiles 테이블의 count를 증가시키는 트리거
CREATE TRIGGER after_bus_insert
AFTER INSERT ON buses
FOR EACH ROW
BEGIN
    IF NEW.deleted = 0 THEN
        UPDATE profiles 
        SET count = count + 1 
        WHERE id = NEW.profile_id;
    END IF;
END//

-- metros 테이블의 deleted 상태가 변경될 때 profiles 테이블의 count를 업데이트하는 트리거
CREATE TRIGGER after_metro_update
AFTER UPDATE ON metros
FOR EACH ROW
BEGIN
    IF NEW.deleted = 1 AND OLD.deleted = 0 THEN
        -- 레코드가 삭제 상태로 변경될 때 count 감소
        UPDATE profiles 
        SET count = count - 1 
        WHERE id = NEW.profile_id;
    ELSEIF NEW.deleted = 0 AND OLD.deleted = 1 THEN
        -- 레코드가 삭제 상태에서 복구될 때 count 증가
        UPDATE profiles 
        SET count = count + 1 
        WHERE id = NEW.profile_id;
    END IF;
END//

-- buses 테이블의 deleted 상태가 변경될 때 profiles 테이블의 count를 업데이트하는 트리거
CREATE TRIGGER after_bus_update
AFTER UPDATE ON buses
FOR EACH ROW
BEGIN
    IF NEW.deleted = 1 AND OLD.deleted = 0 THEN
        -- 레코드가 삭제 상태로 변경될 때 count 감소
        UPDATE profiles 
        SET count = count - 1 
        WHERE id = NEW.profile_id;
    ELSEIF NEW.deleted = 0 AND OLD.deleted = 1 THEN
        -- 레코드가 삭제 상태에서 복구될 때 count 증가
        UPDATE profiles 
        SET count = count + 1 
        WHERE id = NEW.profile_id;
    END IF;
END//

-- 물리적 삭제에 대한 트리거
CREATE TRIGGER before_metro_delete
BEFORE DELETE ON metros
FOR EACH ROW
BEGIN
    IF OLD.deleted = 0 THEN
        UPDATE profiles 
        SET count = count - 1 
        WHERE id = OLD.profile_id;
    END IF;
END//

CREATE TRIGGER before_bus_delete
BEFORE DELETE ON buses
FOR EACH ROW
BEGIN
    IF OLD.deleted = 0 THEN
        UPDATE profiles 
        SET count = count - 1 
        WHERE id = OLD.profile_id;
    END IF;
END//

DELIMITER ;

-- 전체 노선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('1호선', '소요산'),
('1호선', '동두천'),
('1호선', '보산'),
('1호선', '동두천중앙'),
('1호선', '지행'),
('1호선', '덕정'),
('1호선', '덕계'),
('1호선', '양주'),
('1호선', '녹양'),
('1호선', '가능'),
('1호선', '의정부'),
('1호선', '회룡'),
('1호선', '망월사'),
('1호선', '도봉산'),
('1호선', '도봉'),
('1호선', '방학'),
('1호선', '창동'),
('1호선', '녹천'),
('1호선', '월계'),
('1호선', '광운대'),
('1호선', '석계'),
('1호선', '신이문'),
('1호선', '외대앞'),
('1호선', '회기'),
('1호선', '청량리'),
('1호선', '제기동'),
('1호선', '신설동'),
('1호선', '동묘앞'),
('1호선', '동대문'),
('1호선', '종로5가'),
('1호선', '종로3가'),
('1호선', '종각'),
('1호선', '시청'),
('1호선', '서울'),
('1호선', '남영'),
('1호선', '용산'),
('1호선', '노량진'),
('1호선', '대방'),
('1호선', '신길'),
('1호선', '영등포'),
('1호선', '신도림'),
('1호선', '구로'),
('1호선', '구일'),
('1호선', '개봉'),
('1호선', '오류동'),
('1호선', '온수'),
('1호선', '역곡'),
('1호선', '소사'),
('1호선', '부천'),
('1호선', '중동'),
('1호선', '송내'),
('1호선', '부개'),
('1호선', '부평'),
('1호선', '백운'),
('1호선', '동암'),
('1호선', '간석'),
('1호선', '주안'),
('1호선', '도화'),
('1호선', '제물포'),
('1호선', '도원'),
('1호선', '동인천'),
('1호선', '인천'),
('1호선', '청산'),
('1호선', '전곡'),
('1호선', '연천'),
('1호선', '광명'),
('1호선', '가산디지털단지'),
('1호선', '독산'),
('1호선', '금천구청'),
('1호선', '석수'),
('1호선', '관악'),
('1호선', '안양'),
('1호선', '명학'),
('1호선', '금정'),
('1호선', '군포'),
('1호선', '당정'),
('1호선', '의왕'),
('1호선', '성균관대'),
('1호선', '화서'),
('1호선', '수원'),
('1호선', '세류'),
('1호선', '병점'),
('1호선', '세마'),
('1호선', '오산대'),
('1호선', '오산'),
('1호선', '진위'),
('1호선', '송탄'),
('1호선', '서정리'),
('1호선', '지제'),
('1호선', '평택'),
('1호선', '성환'),
('1호선', '직산'),
('1호선', '두정'),
('1호선', '천안'),
('1호선', '봉명'),
('1호선', '쌍용(나사렛대)'),
('1호선', '아산'),
('1호선', '탕정'),
('1호선', '배방'),
('1호선', '온양온천'),
('1호선', '신창'),
('1호선', '서동탄');

-- 2호선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('2호선', '시청'),
('2호선', '을지로입구'),
('2호선', '을지로3가'),
('2호선', '을지로4가'),
('2호선', '동대문역사문화공원'),
('2호선', '신당'),
('2호선', '상왕십리'),
('2호선', '왕십리'),
('2호선', '한양대'),
('2호선', '뚝섬'),
('2호선', '성수'),
('2호선', '건대입구'),
('2호선', '구의'),
('2호선', '강변'),
('2호선', '잠실나루'),
('2호선', '잠실'),
('2호선', '잠실새내'),
('2호선', '종합운동장'),
('2호선', '삼성'),
('2호선', '선릉'),
('2호선', '역삼'),
('2호선', '강남'),
('2호선', '교대'),
('2호선', '서초'),
('2호선', '방배'),
('2호선', '사당'),
('2호선', '낙성대'),
('2호선', '서울대입구'),
('2호선', '봉천'),
('2호선', '신림'),
('2호선', '신대방'),
('2호선', '구로디지털단지'),
('2호선', '대림'),
('2호선', '신도림'),
('2호선', '문래'),
('2호선', '영등포구청'),
('2호선', '당산'),
('2호선', '합정'),
('2호선', '홍대입구'),
('2호선', '신촌'),
('2호선', '이대'),
('2호선', '아현'),
('2호선', '충정로'),
('2호선', '용답'),
('2호선', '신답'),
('2호선', '용두'),
('2호선', '신설동'),
('2호선', '도림천'),
('2호선', '양천구청'),
('2호선', '신정네거리'),
('2호선', '까치산');

-- 3호선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('3호선', '대화'),
('3호선', '주엽'),
('3호선', '정발산'),
('3호선', '마두'),
('3호선', '백석'),
('3호선', '대곡'),
('3호선', '화정'),
('3호선', '원당'),
('3호선', '원흥'),
('3호선', '삼송'),
('3호선', '지축'),
('3호선', '구파발'),
('3호선', '연신내'),
('3호선', '불광'),
('3호선', '녹번'),
('3호선', '홍제'),
('3호선', '무악재'),
('3호선', '독립문'),
('3호선', '경복궁'),
('3호선', '안국'),
('3호선', '종로3가'),
('3호선', '을지로3가'),
('3호선', '충무로'),
('3호선', '동대입구'),
('3호선', '약수'),
('3호선', '금호'),
('3호선', '옥수'),
('3호선', '압구정'),
('3호선', '신사'),
('3호선', '잠원'),
('3호선', '고속터미널'),
('3호선', '교대'),
('3호선', '남부터미널'),
('3호선', '양재'),
('3호선', '매봉'),
('3호선', '도곡'),
('3호선', '대치'),
('3호선', '학여울'),
('3호선', '대청'),
('3호선', '일원'),
('3호선', '수서'),
('3호선', '가락시장'),
('3호선', '경찰병원'),
('3호선', '오금');

-- 4호선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('4호선', '불암산'),
('4호선', '상계'),
('4호선', '노원'),
('4호선', '창동'),
('4호선', '쌍문'),
('4호선', '수유'),
('4호선', '미아'),
('4호선', '미아사거리'),
('4호선', '길음'),
('4호선', '성신여대입구'),
('4호선', '한성대입구'),
('4호선', '혜화'),
('4호선', '동대문'),
('4호선', '동대문역사문화공원'),
('4호선', '충무로'),
('4호선', '명동'),
('4호선', '회현'),
('4호선', '서울'),
('4호선', '숙대입구'),
('4호선', '삼각지'),
('4호선', '신용산'),
('4호선', '이촌'),
('4호선', '동작'),
('4호선', '총신대입구(이수)'),
('4호선', '사당'),
('4호선', '남태령'),
('4호선', '선바위'),
('4호선', '경마공원'),
('4호선', '대공원'),
('4호선', '과천'),
('4호선', '정부과천청사'),
('4호선', '인덕원'),
('4호선', '평촌'),
('4호선', '범계'),
('4호선', '금정'),
('4호선', '산본'),
('4호선', '수리산'),
('4호선', '대야미'),
('4호선', '반월'),
('4호선', '상록수'),
('4호선', '한대앞'),
('4호선', '중앙'),
('4호선', '고잔'),
('4호선', '초지'),
('4호선', '안산'),
('4호선', '신길온천'),
('4호선', '정왕'),
('4호선', '오이도');

-- 5호선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('5호선', '방화'),
('5호선', '개화산'),
('5호선', '김포공항'),
('5호선', '송정'),
('5호선', '마곡'),
('5호선', '발산'),
('5호선', '우장산'),
('5호선', '화곡'),
('5호선', '까치산'),
('5호선', '신정(은행정)'),
('5호선', '목동'),
('5호선', '오목교(목동운동장앞)'),
('5호선', '양평'),
('5호선', '영등포구청'),
('5호선', '영등포시장'),
('5호선', '신길'),
('5호선', '여의도'),
('5호선', '여의나루'),
('5호선', '마포'),
('5호선', '공덕'),
('5호선', '애오개'),
('5호선', '충정로'),
('5호선', '서대문'),
('5호선', '광화문'),
('5호선', '종로3가'),
('5호선', '을지로4가'),
('5호선', '동대문역사문화공원'),
('5호선', '청구'),
('5호선', '신금호'),
('5호선', '행당'),
('5호선', '왕십리'),
('5호선', '마장'),
('5호선', '답십리'),
('5호선', '장한평'),
('5호선', '군자(능동)'),
('5호선', '아차산(어린이대공원후문)'),
('5호선', '광나루(장신대)'),
('5호선', '천호(풍납토성)'),
('5호선', '강동'),
('5호선', '길동'),
('5호선', '굽은다리(강동구민회관앞)'),
('5호선', '명일'),
('5호선', '고덕'),
('5호선', '상일동'),
('5호선', '강일'),
('5호선', '미사'),
('5호선', '하남풍산'),
('5호선', '하남시청'),
('5호선', '하남검단산'),
('5호선', '둔촌동'),
('5호선', '올림픽공원(한국체대)'),
('5호선', '방이'),
('5호선', '오금'),
('5호선', '개롱'),
('5호선', '거여'),
('5호선', '마천');

-- 6호선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('6호선', '응암순환(상선)'),
('6호선', '역촌'),
('6호선', '불광'),
('6호선', '독바위'),
('6호선', '연신내'),
('6호선', '구산'),
('6호선', '새절(신사)'),
('6호선', '증산(명지대앞)'),
('6호선', '디지털미디어시티'),
('6호선', '월드컵경기장(성산)'),
('6호선', '마포구청'),
('6호선', '망원'),
('6호선', '합정'),
('6호선', '상수'),
('6호선', '광흥창'),
('6호선', '대흥(서강대앞)'),
('6호선', '공덕'),
('6호선', '효창공원앞'),
('6호선', '삼각지'),
('6호선', '녹사평'),
('6호선', '이태원'),
('6호선', '한강진'),
('6호선', '버티고개'),
('6호선', '약수'),
('6호선', '청구'),
('6호선', '신당'),
('6호선', '동묘앞'),
('6호선', '창신'),
('6호선', '보문'),
('6호선', '안암(고대병원앞)'),
('6호선', '고려대'),
('6호선', '월곡(동덕여대)'),
('6호선', '상월곡(한국과학기술연구원)'),
('6호선', '돌곶이'),
('6호선', '석계'),
('6호선', '태릉입구'),
('6호선', '화랑대(서울여대입구)'),
('6호선', '봉화산'),
('6호선', '신내');

-- 7호선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('7호선', '장암'),
('7호선', '도봉산'),
('7호선', '수락산'),
('7호선', '마들'),
('7호선', '노원'),
('7호선', '중계'),
('7호선', '하계'),
('7호선', '공릉(서울산업대입구)'),
('7호선', '태릉입구'),
('7호선', '먹골'),
('7호선', '중화'),
('7호선', '상봉'),
('7호선', '면목'),
('7호선', '사가정'),
('7호선', '용마산'),
('7호선', '중곡'),
('7호선', '군자(능동)'),
('7호선', '어린이대공원(세종대)'),
('7호선', '건대입구'),
('7호선', '뚝섬유원지'),
('7호선', '청담'),
('7호선', '강남구청'),
('7호선', '학동'),
('7호선', '논현'),
('7호선', '반포'),
('7호선', '고속터미널'),
('7호선', '내방'),
('7호선', '총신대입구(이수)'),
('7호선', '남성'),
('7호선', '숭실대입구(살피재)'),
('7호선', '상도(중앙대앞)'),
('7호선', '장승배기'),
('7호선', '신대방삼거리'),
('7호선', '보라매'),
('7호선', '신풍'),
('7호선', '대림'),
('7호선', '남구로'),
('7호선', '가산디지털단지'),
('7호선', '철산'),
('7호선', '광명사거리'),
('7호선', '천왕'),
('7호선', '온수'),
('7호선', '까치울'),
('7호선', '부천종합운동장'),
('7호선', '춘의'),
('7호선', '신중동'),
('7호선', '부천시청'),
('7호선', '상동'),
('7호선', '삼산체육관'),
('7호선', '굴포천'),
('7호선', '부평구청'),
('7호선', '산곡'),
('7호선', '석남');

-- 8호선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('8호선', '별내'),
('8호선', '다산'),
('8호선', '동구릉'),
('8호선', '구리'),
('8호선', '장자호수공원'),
('8호선', '암사역사공원'),
('8호선', '암사'),
('8호선', '천호(풍납토성)'),
('8호선', '강동구청'),
('8호선', '몽촌토성(평화의문)'),
('8호선', '잠실'),
('8호선', '석촌'),
('8호선', '송파'),
('8호선', '가락시장'),
('8호선', '문정'),
('8호선', '장지'),
('8호선', '복정'),
('8호선', '남위례'),
('8호선', '산성'),
('8호선', '남한산성입구(성남법원,검찰청)'),
('8호선', '단대오거리'),
('8호선', '신흥'),
('8호선', '수진'),
('8호선', '모란');

-- 9호선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('9호선', '개화'),
('9호선', '김포공항'),
('9호선', '공항시장'),
('9호선', '신방화'),
('9호선', '마곡나루'),
('9호선', '양천향교'),
('9호선', '가양'),
('9호선', '증미'),
('9호선', '등촌'),
('9호선', '염창'),
('9호선', '신목동'),
('9호선', '선유도'),
('9호선', '당산'),
('9호선', '국회의사당'),
('9호선', '여의도'),
('9호선', '샛강'),
('9호선', '노량진'),
('9호선', '노들'),
('9호선', '흑석'),
('9호선', '동작'),
('9호선', '구반포'),
('9호선', '신반포'),
('9호선', '고속터미널'),
('9호선', '사평'),
('9호선', '신논현'),
('9호선', '언주'),
('9호선', '선정릉'),
('9호선', '삼성중앙'),
('9호선', '봉은사'),
('9호선', '종합운동장'),
('9호선', '삼전'),
('9호선', '석촌고분'),
('9호선', '석촌'),
('9호선', '송파나루'),
('9호선', '한성백제'),
('9호선', '올림픽공원'),
('9호선', '둔촌오륜'),
('9호선', '중앙보훈병원');

-- GTX-A 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('GTX-A', '운정중앙'),
('GTX-A', '킨텍스'),
('GTX-A', '대곡'),
('GTX-A', '연신내'),
('GTX-A', '서울역'),
('GTX-A', '수서'),
('GTX-A', '성남'),
('GTX-A', '구성'),
('GTX-A', '동탄');

-- 경의중앙선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('경의중앙선', '용산'),
('경의중앙선', '이촌'),
('경의중앙선', '서빙고'),
('경의중앙선', '한남'),
('경의중앙선', '옥수'),
('경의중앙선', '응봉'),
('경의중앙선', '왕십리'),
('경의중앙선', '청량리'),
('경의중앙선', '회기'),
('경의중앙선', '중랑'),
('경의중앙선', '상봉'),
('경의중앙선', '망우'),
('경의중앙선', '양원'),
('경의중앙선', '구리'),
('경의중앙선', '도농'),
('경의중앙선', '양정'),
('경의중앙선', '덕소'),
('경의중앙선', '도심'),
('경의중앙선', '팔당'),
('경의중앙선', '운길산'),
('경의중앙선', '양수'),
('경의중앙선', '신원'),
('경의중앙선', '국수'),
('경의중앙선', '아신'),
('경의중앙선', '오빈'),
('경의중앙선', '양평'),
('경의중앙선', '원덕'),
('경의중앙선', '용문'),
('경의중앙선', '지평'),
('경의중앙선', '공덕'),
('경의중앙선', '서강대'),
('경의중앙선', '홍대입구'),
('경의중앙선', '가좌'),
('경의중앙선', '디지털미디어시티'),
('경의중앙선', '수색'),
('경의중앙선', '한국항공대'),
('경의중앙선', '강매'),
('경의중앙선', '행신'),
('경의중앙선', '능곡'),
('경의중앙선', '대곡'),
('경의중앙선', '곡산'),
('경의중앙선', '백마'),
('경의중앙선', '풍산'),
('경의중앙선', '일산'),
('경의중앙선', '탄현'),
('경의중앙선', '야당'),
('경의중앙선', '운정'),
('경의중앙선', '금릉'),
('경의중앙선', '금촌'),
('경의중앙선', '월롱'),
('경의중앙선', '파주'),
('경의중앙선', '문산'),
('경의중앙선', '운천'),
('경의중앙선', '임진강'),
('경의중앙선', '효창공원앞'),
('경의중앙선', '신촌(경의중앙선)'),
('경의중앙선', '서울');

-- 공항철도 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('공항철도', '공덕'),
('공항철도', '홍대입구'),
('공항철도', '디지털미디어시티'),
('공항철도', '김포공항'),
('공항철도', '계양'),
('공항철도', '검암'),
('공항철도', '운서'),
('공항철도', '공항화물청사'),
('공항철도', '인천공항1터미널'),
('공항철도', '인천공항2터미널'),
('공항철도', '마곡나루'),
('공항철도', '청라국제도시'),
('공항철도', '영종');

-- 경춘선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('경춘선', '청량리'),
('경춘선', '회기'),
('경춘선', '중랑'),
('경춘선', '광운대'),
('경춘선', '상봉'),
('경춘선', '망우'),
('경춘선', '신내'),
('경춘선', '갈매'),
('경춘선', '별내'),
('경춘선', '퇴계원'),
('경춘선', '사릉'),
('경춘선', '금곡'),
('경춘선', '평내호평'),
('경춘선', '천마산'),
('경춘선', '마석'),
('경춘선', '대성리'),
('경춘선', '청평'),
('경춘선', '상천'),
('경춘선', '가평'),
('경춘선', '굴봉산'),
('경춘선', '백양리'),
('경춘선', '강촌'),
('경춘선', '김유정'),
('경춘선', '남춘천'),
('경춘선', '춘천');

-- 수인분당선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('수인분당선', '청량리'),
('수인분당선', '왕십리'),
('수인분당선', '서울숲'),
('수인분당선', '압구정로데오'),
('수인분당선', '강남구청'),
('수인분당선', '선정릉'),
('수인분당선', '선릉'),
('수인분당선', '한티'),
('수인분당선', '도곡'),
('수인분당선', '구룡'),
('수인분당선', '개포동'),
('수인분당선', '대모산입구'),
('수인분당선', '수서'),
('수인분당선', '복정'),
('수인분당선', '가천대'),
('수인분당선', '태평'),
('수인분당선', '모란'),
('수인분당선', '야탑'),
('수인분당선', '이매'),
('수인분당선', '서현'),
('수인분당선', '수내'),
('수인분당선', '정자'),
('수인분당선', '미금'),
('수인분당선', '오리'),
('수인분당선', '죽전'),
('수인분당선', '보정'),
('수인분당선', '구성'),
('수인분당선', '신갈'),
('수인분당선', '기흥'),
('수인분당선', '상갈'),
('수인분당선', '청명'),
('수인분당선', '영통'),
('수인분당선', '망포'),
('수인분당선', '매탄권선'),
('수인분당선', '수원시청'),
('수인분당선', '매교'),
('수인분당선', '수원'),
('수인분당선', '고색'),
('수인분당선', '오목천'),
('수인분당선', '어천'),
('수인분당선', '야목'),
('수인분당선', '사리'),
('수인분당선', '한대앞'),
('수인분당선', '중앙'),
('수인분당선', '고잔'),
('수인분당선', '초지'),
('수인분당선', '안산'),
('수인분당선', '신길온천'),
('수인분당선', '정왕'),
('수인분당선', '오이도'),
('수인분당선', '달월'),
('수인분당선', '월곶'),
('수인분당선', '소래포구'),
('수인분당선', '인천논현'),
('수인분당선', '호구포'),
('수인분당선', '남동인더스파크'),
('수인분당선', '원인재'),
('수인분당선', '연수'),
('수인분당선', '송도'),
('수인분당선', '인하대'),
('수인분당선', '숭의'),
('수인분당선', '신포'),
('수인분당선', '인천');

-- 신분당선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('신분당선', '신사'),
('신분당선', '논현'),
('신분당선', '신논현'),
('신분당선', '강남'),
('신분당선', '양재'),
('신분당선', '양재시민의숲'),
('신분당선', '청계산입구'),
('신분당선', '판교'),
('신분당선', '정자'),
('신분당선', '미금'),
('신분당선', '동천'),
('신분당선', '수지구청'),
('신분당선', '성복'),
('신분당선', '상현'),
('신분당선', '광교중앙'),
('신분당선', '광교');

-- 경강선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('경강선', '판교'),
('경강선', '성남'),
('경강선', '이매'),
('경강선', '삼동'),
('경강선', '경기광주'),
('경강선', '초월'),
('경강선', '곤지암'),
('경강선', '신둔도예촌'),
('경강선', '이천'),
('경강선', '부발'),
('경강선', '세종왕릉'),
('경강선', '여주');

-- 우이신설선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('우이신설선', '북한산우이'),
('우이신설선', '솔밭공원'),
('우이신설선', '4.19 민주묘지'),
('우이신설선', '가오리'),
('우이신설선', '화계'),
('우이신설선', '삼양'),
('우이신설선', '삼양사거리'),
('우이신설선', '솔샘'),
('우이신설선', '북한산보국문'),
('우이신설선', '정릉'),
('우이신설선', '성신여대입구'),
('우이신설선', '보문'),
('우이신설선', '신설동');

-- 서해선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('서해선', '일산'),
('서해선', '풍산'),
('서해선', '백마'),
('서해선', '곡산'),
('서해선', '대곡'),
('서해선', '능곡'),
('서해선', '김포공항'),
('서해선', '원종'),
('서해선', '부천종합운동장'),
('서해선', '소사'),
('서해선', '소새울'),
('서해선', '시흥대야'),
('서해선', '신천'),
('서해선', '신현'),
('서해선', '시흥시청'),
('서해선', '시흥능곡'),
('서해선', '달미'),
('서해선', '선부'),
('서해선', '초지'),
('서해선', '시우'),
('서해선', '원시');

-- 신림선 데이터 INSERT
INSERT INTO metro_informations (line_number, station_name) VALUES 
('신림선', '샛강'),
('신림선', '대방'),
('신림선', '서울지방병무청'),
('신림선', '보라매'),
('신림선', '보라매공원'),
('신림선', '보라매병원'),
('신림선', '당곡'),
('신림선', '신림'),
('신림선', '서원'),
('신림선', '서울대벤처타운'),
('신림선', '관악산');

ALTER TABLE businformations ADD INDEX idx_route_id (route_id);
ALTER TABLE businformations ADD INDEX idx_route (route);
ALTER TABLE businformations ADD INDEX idx_station_id (station_id);
ALTER TABLE businformations ADD INDEX idx_route_station (route, station);
ALTER TABLE businformations ADD INDEX idx_route_sequence (route, sequence);