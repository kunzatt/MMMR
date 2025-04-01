package com.ssafy.mmmr.news.entity;

import static com.querydsl.core.types.PathMetadataFactory.*;

import com.querydsl.core.types.dsl.*;

import com.querydsl.core.types.PathMetadata;
import javax.annotation.processing.Generated;
import com.querydsl.core.types.Path;


/**
 * QNewsEntity is a Querydsl query type for NewsEntity
 */
@Generated("com.querydsl.codegen.DefaultEntitySerializer")
public class QNewsEntity extends EntityPathBase<NewsEntity> {

    private static final long serialVersionUID = -1304814456L;

    public static final QNewsEntity newsEntity = new QNewsEntity("newsEntity");

    public final StringPath content = createString("content");

    public final DateTimePath<java.time.LocalDateTime> created_at = createDateTime("created_at", java.time.LocalDateTime.class);

    public final NumberPath<Long> id = createNumber("id", Long.class);

    public final StringPath title = createString("title");

    public final DateTimePath<java.time.LocalDateTime> updated_at = createDateTime("updated_at", java.time.LocalDateTime.class);

    public QNewsEntity(String variable) {
        super(NewsEntity.class, forVariable(variable));
    }

    public QNewsEntity(Path<? extends NewsEntity> path) {
        super(path.getType(), path.getMetadata());
    }

    public QNewsEntity(PathMetadata metadata) {
        super(NewsEntity.class, metadata);
    }

}

