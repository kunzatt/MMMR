package com.ssafy.mmmr.news.repository;

import com.ssafy.mmmr.news.entity.NewsEntity;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.Optional;

@Repository
public interface NewsRepository extends JpaRepository<NewsEntity, Long> {
     void create(NewsEntity newsEntity);
}
